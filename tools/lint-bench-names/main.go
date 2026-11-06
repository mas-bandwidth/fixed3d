// lint-bench-names refuses internal host names, addresses and home-directory paths in
// tracked files.
//
// fixed3d is public. Benches are named neutrally (bench-linux-a, bench-mac-a, ...), and
// paths on a bench are written relative to a neutral root (/work/...). The check reads
// every tracked text file outside extern/ and fails on:
//   - a path under a user's home directory: /Users/<name>, /home/<name>, C:\Users\<name>
//     (the placeholder USER passes)
//   - an mDNS host name: <name>.local, any case
//   - a user@host login whose host has no domain, or is an IPv4 address (an email address
//     passes; so do an action pin such as checkout@v4 and the neutral user@bench-... form)
//   - a Tailscale (100.64.0.0/10) or private LAN (192.168.0.0/16, 10.0.0.0/8) IPv4 address
//   - any token, or any hyphen-separated prefix of a token, whose SHA-256 (of the
//     lowercased text) is listed in denied.sha256
//
// The hashes keep the names out of plain text in this repository. They are not secret:
// short names fall to guessing.
//
// The tool's own directory is not scanned: its self-test lines must hold the forms it refuses.
//
// Run from the repository root:
//   go run tools/lint-bench-names/main.go              check the tracked files
//   go run tools/lint-bench-names/main.go -self-test   check that each rule fires
package main

import (
	"bufio"
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"fmt"
	"os"
	"os/exec"
	"regexp"
	"strconv"
	"strings"
)

var (
	homePath = regexp.MustCompile(`(?i)(/Users|/home|[A-Z]:\\Users)[/\\]([A-Za-z_][A-Za-z0-9._-]*)`)
	mdnsHost = regexp.MustCompile(`(?i)\b[a-z][a-z0-9-]*\.local\b`)
	login    = regexp.MustCompile(`(?i)\b[a-z_][a-z0-9_-]*@([a-z0-9][a-z0-9-]*)((?:\.[a-z0-9-]+)*)`)
	ipv4     = regexp.MustCompile(`\b(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})\b`)
	version  = regexp.MustCompile(`^v[0-9]+$`)
	digits   = regexp.MustCompile(`^[0-9]+$`)
	token    = regexp.MustCompile(`[A-Za-z0-9-]+`)
)

// The self-test's synthetic host. Its hash is added to the denied set only in -self-test,
// so the test names no real machine.
const selfTestHost = "examplehost-9"

func main() {
	denied, err := readDenied("tools/lint-bench-names/denied.sha256")
	if err != nil {
		fmt.Fprintln(os.Stderr, "lint-bench-names:", err)
		os.Exit(2)
	}
	if len(os.Args) > 1 && os.Args[1] == "-self-test" {
		os.Exit(selfTest(denied))
	}
	out, err := exec.Command("git", "ls-files", "-z").Output()
	if err != nil {
		fmt.Fprintln(os.Stderr, "lint-bench-names: git ls-files:", err)
		os.Exit(2)
	}
	failures := 0
	for _, path := range strings.Split(strings.TrimRight(string(out), "\x00"), "\x00") {
		if path == "" || strings.HasPrefix(path, "extern/") || strings.HasPrefix(path, "tools/lint-bench-names/") {
			continue
		}
		data, err := os.ReadFile(path)
		if err != nil {
			fmt.Printf("%s: unreadable: %v\n", path, err)
			failures++
			continue
		}
		if bytes.IndexByte(data, 0) >= 0 {
			continue // binary
		}
		failures += checkFile(path, data, denied)
	}
	if failures > 0 {
		fmt.Fprintf(os.Stderr, "lint-bench-names: %d finding(s). Use a neutral bench name (bench-linux-a, bench-mac-a) and a neutral path (/work/...).\n", failures)
		os.Exit(1)
	}
}

// selfTest runs every rule on one synthetic line that must fire it, and on lines that
// must pass. It is the lint's standing negative control.
func selfTest(denied map[string]bool) int {
	sum := sha256.Sum256([]byte(selfTestHost))
	denied[hex.EncodeToString(sum[:])] = true
	mustFire := map[string]string{
		"home-directory path": "built in /Users/someone/src",
		"windows home path":   `built in C:\Users\someone\src`,
		"mDNS host name":      "ran on Example-Box.LOCAL today",
		"user@host login":     "logged in as someone@examplebox",
		"user@ip login":       "logged in as someone@10.1.2.3",
		"tailscale address":   "reached 100.70.1.2 over the tailnet",
		"lan address":         "reached 192.168.4.5 on the lan",
		"hashed host name":    "ran on " + selfTestHost + " overnight",
		"hashed host suffix":  "ran on " + selfTestHost + "-2 overnight",
	}
	mustPass := []string{
		"uses: actions/checkout@v4",
		"Co-Authored-By: Claude <noreply@anthropic.com>",
		"git clone git@github.com:mas-bandwidth/fixed3d.git",
		"Path: /Users/USER/*/benchmark",
		"ran on user@bench-linux-b in /work/x",
		"version 1.2.3.4 of the spec",
	}
	failed := 0
	for name, line := range mustFire {
		if checkFile("self-test", []byte(line), denied) == 0 {
			fmt.Printf("self-test: rule %q did not fire on %q\n", name, line)
			failed++
		}
	}
	for _, line := range mustPass {
		if checkFile("self-test", []byte(line), denied) != 0 {
			fmt.Printf("self-test: %q should pass\n", line)
			failed++
		}
	}
	if failed > 0 {
		return 1
	}
	fmt.Println("self-test: every rule fires, and the allowed forms pass")
	return 0
}

func privateIP(m []string) bool {
	var o [4]int
	for i := 0; i < 4; i++ {
		v, err := strconv.Atoi(m[i+1])
		if err != nil || v > 255 {
			return false
		}
		o[i] = v
	}
	return (o[0] == 100 && o[1] >= 64 && o[1] <= 127) || (o[0] == 192 && o[1] == 168) || o[0] == 10
}

func checkFile(path string, data []byte, denied map[string]bool) int {
	failures := 0
	scanner := bufio.NewScanner(bytes.NewReader(data))
	scanner.Buffer(make([]byte, 1024*1024), 64*1024*1024)
	n := 0
	for scanner.Scan() {
		n++
		line := scanner.Text()
		report := func(what string) {
			fmt.Printf("%s:%d: %s\n", path, n, what)
			failures++
		}
		for _, m := range homePath.FindAllStringSubmatch(line, -1) {
			if m[2] != "USER" {
				report("home-directory path")
			}
		}
		if mdnsHost.MatchString(line) {
			report("mDNS host name")
		}
		for _, m := range login.FindAllStringSubmatch(line, -1) {
			host, domain := strings.ToLower(m[1]), m[2]
			isIP := digits.MatchString(host) && strings.Count(domain, ".") == 3
			if isIP || (domain == "" && !version.MatchString(host) && !strings.HasPrefix(host, "bench-")) {
				report("user@host login")
			}
		}
		for _, m := range ipv4.FindAllStringSubmatch(line, -1) {
			if privateIP(m) {
				report("private or tailnet address")
			}
		}
		for _, t := range token.FindAllString(line, -1) {
			parts := strings.Split(strings.ToLower(t), "-")
			for i := 1; i <= len(parts); i++ {
				sum := sha256.Sum256([]byte(strings.Join(parts[:i], "-")))
				if denied[hex.EncodeToString(sum[:])] {
					report("internal host name")
					break
				}
			}
		}
	}
	if err := scanner.Err(); err != nil {
		fmt.Printf("%s: not fully read: %v\n", path, err)
		failures++
	}
	return failures
}

func readDenied(path string) (map[string]bool, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	denied := map[string]bool{}
	for _, line := range strings.Split(string(data), "\n") {
		line = strings.TrimSpace(line)
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		denied[line] = true
	}
	return denied, nil
}
