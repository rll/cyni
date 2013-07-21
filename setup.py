import os
import os.path
from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy

openni2_include = os.getenv('OPENNI2_INCLUDE')
openni2_lib = os.getenv('OPENNI2_REDIST')
has_emitter_control = os.getenv('OPENNI2_HAS_EMITTER_CONTROL', 0)
has_emitter_control = bool(has_emitter_control)

class build_ext_with_config(build_ext):
    def build_extensions(self):
        print 'Generate config.pxi'
        filename = os.path.join(os.path.dirname(__file__), 'config.pxi')
        with open(filename, 'w') as fd:
                for k, v in c_options.iteritems():
                            fd.write('DEF %s = %d\n' % (k.upper(), int(v)))
        build_ext.build_extensions(self)
        os.remove(filename)
        cppfilename = os.path.join(os.path.dirname(__file__), 'cyni.cpp')
        os.remove(cppfilename)

c_options = {
        'has_emitter_control': has_emitter_control,
        }

ext_modules = [
    Extension(
        "cyni",
        ["cyni.pyx"],
        language="c++",
        include_dirs=[openni2_include, numpy.get_include()],
        libraries=['OpenNI2'],
        library_dirs=[openni2_lib],
        runtime_library_dirs=[openni2_lib],
    )
]
setup(
  name = 'cyni',
  version='0.0.1',
  cmdclass = {'build_ext': build_ext_with_config},
  ext_modules = ext_modules
)
