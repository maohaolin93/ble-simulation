# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

# def options(opt):
#     pass

# def configure(conf):
#     conf.check_nonfatal(header_name='stdint.h', define_name='HAVE_STDINT_H')

def build(bld):
    module = bld.create_ns3_module('ble', ['core', 'network', 'mobility',
    'spectrum','propagation', 'energy'])
    module.source = [
        'model/ble-error-model.cc',
        'model/ble-phy.cc',
        'model/ble-spectrum-signal-parameters.cc',
        'model/ble-net-device.cc',
        'model/ble-bb-manager.cc',
        'model/ble-link.cc',
        'model/ble-link-controller.cc',
        'model/ble-link-manager.cc',
        'model/ble-mac-header.cc',
        'model/ble-application.cc',
        'helper/ble-helper.cc',
      #  'helper/ble-helper-lorabased.cc',
        ]

    module_test = bld.create_ns3_module_test_library('ble')
    module_test.source = [
        'test/ble-test-suite.cc',
        'test/ble-test-suite-broadcast.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'ble'
    headers.source = [
        'model/constants.h',
        'model/ble-error-model.h',
        'model/ble-phy.h',
        'model/ble-spectrum-signal-parameters.h',
        'model/ble-net-device.h',
        'model/ble-bb-manager.h',
        'model/ble-link.h',
        'model/ble-link-controller.h',
        'model/ble-link-manager.h',
        'model/ble-mac-header.h',
        'model/ble-application.h',
        'helper/ble-helper.h',
        #'helper/ble-helper-lorabased.h',
        ]

    if bld.env.ENABLE_EXAMPLES:
        bld.recurse('examples')

    # bld.ns3_python_bindings()

