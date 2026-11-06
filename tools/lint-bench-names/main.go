// lint-bench-names refuses internal host names and home-directory paths in tracked files.
//
// fixed3d is public. Benches are named neutrally (bench-linux-a, bench-mac-a, ...), and
// paths on a bench are written relative to a neutral root (/work/...). The check reads
// every tracked text file outside extern/ and fails on:
//   - a path under a user's home directory: /Users/<name>/ or /home/<name>/
//   - an mDNS host name: <name>.local
//   - a user@host login with no domain (an email address has a dot after the @ and passes)
//   - any token whose SHA-256 is listed in denied.sha256 (names kept hashed so the list
//     does not publish them)
//
// Run from the repository root: go run tools/lint-bench-names/main.go
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
	"strings"
)

var (
	homePath = regexp.MustCompile(`(/Users|/home)/[A-Za-z_][A-Za-z0-9._-]*/`)
	mdnsHost = regexp.MustCompile(`\b[A-Za-z][A-Za-z0-9-]*\.local\b`)
	login    = regexp.MustCompile(`\b[a-z_][a-z0-9_-]*@[a-z][a-z0-9-]*([.a-z0-9-]?)`)
	token    = regexp.MustCompile(`[A-Za-z0-9-]+`)
)

func main() {
	denied, err := readDenied("tools/lint-bench-names/denied.sha256")
	if err != nil {
		fmt.Fprintln(os.Stderr, "lint-bench-names:", err)
		os.Exit(2)
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
		if err != nil || bytes.IndexByte(data, 0) >= 0 {
			continue // unreadable, or binary
		}
		failures += checkFile(path, data, denied)
	}
	if failures > 0 {
		fmt.Fprintf(os.Stderr, "lint-bench-names: %d finding(s). Use a neutral bench name (bench-linux-a, bench-mac-a) and a neutral path (/work/...).\n", failures)
		os.Exit(1)
	}
}

func checkFile(path string, data []byte, denied map[string]bool) int {
	failures := 0
	scanner := bufio.NewScanner(bytes.NewReader(data))
	scanner.Buffer(make([]byte, 1024*1024), 16*1024*1024)
	for n := 1; scanner.Scan(); n++ {
		line := scanner.Text()
		report := func(what string) {
			fmt.Printf("%s:%d: %s\n", path, n, what)
			failures++
		}
		if homePath.MatchString(line) {
			report("home-directory path")
		}
		if mdnsHost.MatchString(line) {
			report("mDNS host name")
		}
		for _, m := range login.FindAllStringSubmatch(line, -1) {
			if m[1] != "." {
				report("user@host login")
			}
		}
		for _, t := range token.FindAllString(line, -1) {
			sum := sha256.Sum256([]byte(strings.ToLower(t)))
			if denied[hex.EncodeToString(sum[:])] {
				report("internal host name")
			}
		}
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
