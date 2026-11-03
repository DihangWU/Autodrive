"""把实验操作接入现有 SUMO GUI 构建过程。"""
from vanet.project_paths import SUMO_GUI_FEATURE_SOURCE_FILE, SUMO_GUI_WINDOW_SOURCE_FILE


def source_patches():
    code=SUMO_GUI_FEATURE_SOURCE_FILE.read_text()
    return {SUMO_GUI_WINDOW_SOURCE_FILE:[
        ('class LincolnNavigationButton : public FXToggleButton {',code+'\nclass LincolnNavigationButton : public FXToggleButton {'),
        ('lincolnNavigationUI = new LincolnNavigationUI(myStatusbar, this);',
         'lincolnNavigationUI = new LincolnNavigationUI(myStatusbar, this);\n        if (std::getenv("AUTODRIVE_VANET")) new VanetExperimentUI(this, this);')
    ]}
