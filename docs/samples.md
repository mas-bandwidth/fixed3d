# Samples {#samples}
Once you have conquered the HelloWorld example, you should start looking
at Fixed3D's samples application. The samples application is a testing framework and demo
environment. Here are some of the features:
- Camera with pan and zoom
- Mouse dragging of dynamic bodies
- Many samples in a tree view
- GUI for selecting samples, parameter tuning, and debug drawing options
- Pause and single step simulation
- Multithreading and performance data

![Fixed3D Samples](images/samples.png)

The samples application has many examples of Fixed3D usage in the test cases and the
framework itself. I encourage you to explore and tinker with the samples
as you learn Fixed3D.

Note: one sample intentionally diverges from Box3D. **Stacking >
Card House** collapses in Fixed3D while standing in Box3D — not from a
conversion defect, but because its cards are thinner than the solver's
contact tolerances, which makes the outcome chaotic: differences far below
any engineering tolerance (a 0.00003 change in one initial rotation, in
*either* number system) change the ending. It is kept deliberately as a
sensitivity canary. The practical guidance it demonstrates: keep feature
sizes comfortably above the solver tolerances if your content relies on
marginally stable assemblies.

Note: the sample application is written using [sokol](https://github.com/floooh/sokol) and
[imgui](https://github.com/ocornut/imgui).
The samples app is not part of the Fixed3D library. The Fixed3D library is agnostic about rendering.
As shown by the HelloWorld example, you don't need a renderer to use Fixed3D.
