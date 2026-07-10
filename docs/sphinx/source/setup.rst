======================
Setup and installation
======================

Preparation
===========

JSON-LD metamodels
------------------

After cloning this repository, checkout the metamodels to the folder `comp-rob2b/metamodels`:

.. code:: sh

  git clone https://github.com/comp-rob2b/metamodels comp-rob2b/metamodels


Code generator
--------------

The following dependencies are required for the code generator:

* `Python <https://www.python.org/>`_
* `rdflib <https://github.com/RDFLib/rdflib>`_
* `pySHACL <https://github.com/RDFLib/pySHACL>`_
* `numpy <https://numpy.org/>`_
* `Java <https://openjdk.org/>`_
* `Apache Ant <https://ant.apache.org/>`_
* `StringTemplate <https://www.stringtemplate.org/>`_
* `STSTv4 <https://github.com/jsnyders/STSTv4>`_ (from Git!)
* `GNU Make <https://www.gnu.org/software/make/>`_

For convenience, we provide a step-by-step installation guide for the latter two dependencies. Note, that STSTv4 comes with a pre-bundled version of StringTemplate. Hence, the steps for StringTemplate can be considered optional and are only relevant if one plans to use a more recent StringTemplate version.

StringTemplate can either be `installed <https://github.com/antlr/stringtemplate4/blob/master/doc/java.md#installation>`_ from the `pre-compiled version <https://www.stringtemplate.org/download.html>`_ or it can be built from source:

1. Download the latest version and extract it to a directory ``<st>``
2. Change to the directory

  .. code:: sh

    cd <st>

2. For version 4.3.3 execute

  .. code:: sh

    sed "s/1.6/1.8/g" -i build.xml

3. Compile using ``ant``
4. This creates a JAR file ``<jar>`` (e.g. ``ST-4.3.4.jar``) in the ``<st>/dist`` folder

STSTv4 must be `built <https://github.com/jsnyders/STSTv4#install-instructions>`_ from the Git version (to support `nested JSON arrays <https://github.com/jsnyders/STSTv4/commit/6f72c8cc19b773bab015ef9cf58cabd2cb2984c8>`_):

1. Clone the source code:

  .. code:: sh

    git clone https://github.com/jsnyders/STSTv4.git

2. Change into the repository: ``cd STSTv4``
3. Build with ``ant``
4. Copy the launch script template:

  .. code:: sh

    cp stst.sh.init stst.sh

5. Adapt the ``STST_HOME`` variable in the launch script
6. (Optional) To use StringTemplate from above adapt the ``CP`` variable:

  .. code:: sh

    sed "s#\$STST_HOME/lib/ST-4.0.8.jar#<st>/dist/<jar>#g" -i stst.sh

7. Fix the launch script:

  .. code:: sh
    
    sed "s#lib/stst.jar#build/jar/stst.jar#g" -i stst.sh

8. Make the launch script executable:

  .. code:: sh

    chmod +x stst.sh


Generated code
--------------

The following dependencies are required to build and execute the generated code:

* Build dependencies:

  * `CMake <https://cmake.org/>`_
  * `GCC <https://gcc.gnu.org/>`_

* Run-time dependencies:

  * `orocos_kdl <https://github.com/orocos/orocos_kinematics_dynamics>`_
  * `urdfdom_headers <https://github.com/ros/urdfdom_headers>`_
  * `urdfdom <https://github.com/ros/urdfdom>`_
  * `robif2b <https://github.com/rosym-project/robif2b>`_
  * `hddc2b <https://github.com/comp-rob2b/hddc2b/tree/primary-secondary-task>`_

Make sure that the latter dependencies are accessible to the generated CMake script. That can be achieved via a local or system-wide installation, but also using CMake's package registry (to avoid the installation). To this end, both projects can be configured with a ``-DENABLE_PACKAGE_REGISTRY=On`` option.


Building and execution
======================

There are four make targets to build the different scenarios:

* ``sc0a``: Keep the robot's right arm in contact with the table
* ``sc0b``: Keep both robot arms in contact with the table
* ``sc1``: Alignment task using an active mobile base
* ``sc2``: Alignment task using active arms

For instance, to build the latter scenario execute:

.. code:: sh

    make sc2

This target triggers the intermediate code generation, code generation and the final compilation.

Run the code via

.. code:: sh

  cd gen/build
  ./main


Documentation
=============

The following dependencies are required for building the documentation:

* `Sphinx <https://www.sphinx-doc.org>`_
* `Read the Docs Sphinx Theme <https://github.com/readthedocs/sphinx_rtd_theme>`_

Build the documentation with the following command:

.. code:: sh

  make tutorial-html
