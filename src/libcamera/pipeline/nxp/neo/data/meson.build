# SPDX-License-Identifier: CC0-1.0

conf_files = files([
    'config.yaml',
])

neo_pipeline_data_dir = pipeline_data_dir / 'nxp' / 'neo'

install_data(conf_files,
             install_dir : pipeline_data_dir / 'nxp' / 'neo',
             install_tag : 'runtime')

config_h.set('NXP_NEO_PIPELINE_DATA_DIR', '"' + get_option('prefix') / neo_pipeline_data_dir + '"')

summary({
         'NXP_NEO_PIPELINE_DATA_DIR' :
         config_h.get('NXP_NEO_PIPELINE_DATA_DIR'),
        }, section : 'Paths')
