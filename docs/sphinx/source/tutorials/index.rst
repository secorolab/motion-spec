=========
Tutorials
=========

These tutorials follow one generation through the CLI in order:

1. generate model artifacts;
2. build the generated controller;
3. run the build and create a run archive;
4. replay and inspect the result.

They use ``pick_place_single.robmot`` from the sibling
`motion-spec-dsl project <https://github.com/secorolab/motion-spec-dsl>`_. Complete
:doc:`end-to-end` first; the other pages reuse its ``generation/pick-place``
directory.

.. toctree::
   :maxdepth: 1

   end-to-end
   reuse-generation
   inspect-run
