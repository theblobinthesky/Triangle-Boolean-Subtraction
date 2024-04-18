<h1>README</h1>
This repository implements CPU-powered occlusion culling using an octree and triangle subtraction. The majority of the code is triangle boolean subtraction in the 2d-plane.<br>

<h2>How to run</h2>
Run <code>make rund</code> to execute the tests which are mainly for the triangle boolean subtraction. Visualize the test instances using <code>./triangle_visualizer.py "n triangle points" "m standalone points"</code>.<br>

<h2>Caveats</h2>
Note that although the code is mostly correct, it is not completely error-free and most likely not fast enough.