import os.path
from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy

c_options = { 
        'has_emitter_control': True,
        }

print 'Generate config.pxi'
with open(os.path.join(os.path.dirname(__file__), 'config.pxi'), 'w') as fd:
        for k, v in c_options.iteritems():
                    fd.write('DEF %s = %d\n' % (k.upper(), int(v)))

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
  cmdclass = {'build_ext': build_ext},
  ext_modules = ext_modules
)
