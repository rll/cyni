import os.path
from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy

class build_ext_with_config(build_ext):
    def build_extensions(self):
        print 'Generate config.pxi'
        filename = os.path.join(os.path.dirname(__file__), 'config.pxi')
        with open(filename, 'w') as fd:
                for k, v in c_options.iteritems():
                            fd.write('DEF %s = %d\n' % (k.upper(), int(v)))
        build_ext.build_extensions(self)
        os.remove(filename)
        cppfilename = os.path.join(os.path.dirname(__file__), 'cyni2.cpp')
        os.remove(cppfilename)

c_options = { 
        'has_emitter_control': True,
        }

ext_modules = [
    Extension(
        "cyni2", 
        ["cyni2.pyx"], 
        language="c++",
        include_dirs=['/opt/OpenNI2/Include', numpy.get_include()],
        libraries=['OpenNI2'],
        library_dirs=[ '/opt/OpenNI2/Redist'],
        runtime_library_dirs=[ '/opt/OpenNI2/Redist'],
    )
]
setup(
  name = 'cyni2',
  cmdclass = {'build_ext': build_ext_with_config},
  ext_modules = ext_modules
)
