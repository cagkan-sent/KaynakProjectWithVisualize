
from enum import Enum


class Sr2D_parameters(Enum):
    Triggerlaser_0x640001 = {"ConfigId": "Toolbar", "ParamId": "0x640001"}
    TriggerLED_0x640002 = {"ConfigId": "Toolbar", "ParamId": "0x640002"}
    Getimage_GetImage = {"ConfigId": "Toolbar", "ParamId": "GetImage"}
    Teach_0x6D0001 = {"ConfigId": "Toolbar", "ParamId": "0x6D0001"}

    CustomName_0xCC0001 = {"ConfigId": "General", "ParamId": "0xCC0001"}
    Autotrigger_0x510001 = {"ConfigId": "General", "ParamId": "0x510001"}
    Exposuretime_0x680001 = {"ConfigId": "General", "ParamId": "0x680001"}
    Usemanualexposuretime_0xBE0001 = {"ConfigId": "General", "ParamId": "0xBE0001"}
    Flashtime_0x100001 = {"ConfigId": "General", "ParamId": "0x100001"}
    Objectcontrast_0x9F0001 = {"ConfigId": "General", "ParamId": "0x9F0001"}
    ROIMinX_0xC90001 = {"ConfigId": "General", "ParamId": "0xC90001"}
    ROIMaxX_0xC90002 = {"ConfigId": "General", "ParamId": "0xC90002"}
    ROIMinZ_0xC90003 = {"ConfigId": "General", "ParamId": "0xC90003"}
    ROIMaxZ_0xC90004 = {"ConfigId": "General", "ParamId": "0xC90004"}
    ImageTransferActive = {"ConfigId": "General", "ParamId": "ImageTransferActive"}


