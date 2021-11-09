# Copyright 2021 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import pathlib
import re
import sys

import yaml


class ConfigureFileScript(object):

    def __init__(self):
        self._params = {}

    def configure(self, name, data):
        self._params[name] = data

    def text(self, behavior):
        params = ['set({} "{}")'.format(*param) for param in self._params.items()]
        config = ['configure_file(${{INPUT}}/{}.in ${{OUTPUT}} @ONLY)'.format(behavior)]
        return '\n'.join(params + config + [''])


class Specification(object):

    def __init__(self, name, data):
        self._name = name.split('/')
        self._type = data['type'].split('/')
        self._behavior = data['behavior']
        self._qos = {
            'default': data.get('default-qos'),
        }

    def spec_name(self):
        return '/api/' + self.path

    def impl_name(self, remap):
        return remap.get(self.path, self.spec_name())

    def cpp_include_guard(self, package):
        return '__'.join([package] + self._name).upper() + '_HPP_'

    def cpp_namespace(self, package):
        return '::'.join([package] + self._name)

    def cpp_type_header(self):
        path = pathlib.Path(*self._type)
        return str(path.with_name(self.camel_to_snake(path.name)).with_suffix('.hpp'))

    def py_type_import(self):
        return '.'.join(self._type[:-1])

    def cpp_type_name(self):
        return '::'.join(self._type)

    def py_type_name(self):
        return self._type[-1]

    def cpp_type_package(self):
        return self._type[0]

    def cpp_qos_code(self, quality, pubsub):
        if self.behavior != 'topic':
            return ''
        reliability = {
            'default': '',
            'reliable': '.reliable()',
            'best_effort': '.best_effort()'
        }
        durability = {
            'default': '',
            'volatile': '.durability_volatile()',
            'transient_local': '.transient_local()'
        }
        qos = self._qos[quality][pubsub]
        reliability = reliability[qos['reliability']]
        durability = durability[qos['durability']]
        return 'rclcpp::QoS({}){}{}'.format(qos['depth'], reliability, durability)

    def py_qos_depth(self, quality, pubsub):
        if self.behavior != 'topic':
            return ''
        return self._qos[quality][pubsub]['depth']

    def py_qos_reliability(self, quality, pubsub):
        if self.behavior != 'topic':
            return ''
        reliability = {
            'default': 'SYSTEM_DEFAULT',
            'reliable': 'RELIABLE',
            'best_effort': 'BEST_EFFORT'
        }
        return reliability[self._qos[quality][pubsub]['reliability']]

    def py_qos_durability(self, quality, pubsub):
        if self.behavior != 'topic':
            return ''
        durability = {
            'default': 'SYSTEM_DEFAULT',
            'volatile': 'VOLATILE',
            'transient_local': 'TRANSIENT_LOCAL'
        }
        return durability[self._qos[quality][pubsub]['durability']]

    @property
    def path(self):
        return '/'.join(self._name)

    @property
    def behavior(self):
        return self._behavior

    @staticmethod
    def camel_to_snake(text):
        return re.sub('([A-Z])', r'_\1', text).lower().strip('_')

    @staticmethod
    def Load(paths):
        for path in paths:
            for name, data in yaml.safe_load(pathlib.Path(path).read_text()).items():
                yield (name, Specification(name, data))


def configure():

    parser = argparse.ArgumentParser()
    parser.add_argument('--package', required=True)
    parser.add_argument('--build', required=True)
    parser.add_argument('--names', required=True)
    parser.add_argument('--remap', required=True)
    parser.add_argument('--extra', required=True)
    parser.add_argument('--specs', required=True, nargs='+')
    args = parser.parse_args()

    remap = yaml.safe_load(pathlib.Path(args.remap).read_text())
    extra = yaml.safe_load(pathlib.Path(args.extra).read_text())
    specs = dict(Specification.Load(args.specs))
    names = pathlib.Path(args.names).read_text().split(';')
    try:
        apis = [specs[name] for name in names]
    except KeyError as exception:
        sys.exit('The API {} is not defined in the specifications.'.format(exception))

    for api in apis:
        script = ConfigureFileScript()
        script.configure('API_SPEC_NAME',      api.spec_name())
        script.configure('API_IMPL_NAME',      api.impl_name(remap))
        script.configure('CPP_INCLUDE_GUARD',  api.cpp_include_guard(args.package))
        script.configure('CPP_NAMESPACE',      api.cpp_namespace(args.package))
        script.configure('CPP_TYPE_HEADER',    api.cpp_type_header())
        script.configure('CPP_TYPE_NAME',      api.cpp_type_name())
        script.configure('CPP_TYPE_PACKAGE',   api.cpp_type_package())
        script.configure('CPP_QOS_PUBLISH',    api.cpp_qos_code('default', 'pub'))
        script.configure('CPP_QOS_SUBSCRIBE',  api.cpp_qos_code('default', 'sub'))
        script.configure('CPP_SERVICE_HEADER', extra['library-cpp']['service']['header'])
        script.configure('CPP_SERVICE',        extra['library-cpp']['service']['service'])
        script.configure('CPP_CLIENT',         extra['library-cpp']['service']['client'])
        script.configure('CPP_TOPIC_HEADER',   extra['library-cpp']['topic']['header'])
        script.configure('CPP_SUBSCRIPTION',   extra['library-cpp']['topic']['subscription'])
        script.configure('CPP_PUBLISHER',      extra['library-cpp']['topic']['publisher'])
        script.configure('PY_TYPE_IMPORT',         api.py_type_import())
        script.configure('PY_TYPE_NAME',           api.py_type_name())
        script.configure('PY_QOS_PUBLISH_DEPTH',   api.py_qos_depth('default', 'pub'))
        script.configure('PY_QOS_PUBLISH_REL',     api.py_qos_reliability('default', 'pub'))
        script.configure('PY_QOS_PUBLISH_DUR',     api.py_qos_durability('default', 'pub'))
        script.configure('PY_QOS_SUBSCRIBE_DEPTH', api.py_qos_depth('default', 'sub'))
        script.configure('PY_QOS_SUBSCRIBE_REL',   api.py_qos_reliability('default', 'sub'))
        script.configure('PY_QOS_SUBSCRIBE_DUR',   api.py_qos_durability('default', 'sub'))
        path = pathlib.Path(args.build, api.path).with_suffix('.cmake')
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(script.text(api.behavior))


configure()
