package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_a8000m2_lpnFn1_4
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=110.53,165.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=163.56,99.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=158.28,3.64");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=215.02,141.99");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=3.51,29.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=2.33,110.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=227.63,52.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=279.12,108.29");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=294.93,211.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=365.49,209.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=369.54,287.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=288.52,282.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=271.67,360.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=207.14,387.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=262.50,468.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=315.62,551.07");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=279.05,606.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=212.72,587.10");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=341.67,495.18");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=392.39,458.71");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=358.68,391.02");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=209.15,288.81");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=223.24,212.97");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=365.66,142.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=427.44,226.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=509.74,180.37");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=466.88,312.23");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=547.00,349.56");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=555.05,411.04");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=604.59,460.15");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=658.52,514.84");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=690.41,592.34");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=785.51,603.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=764.56,671.59");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=857.61,658.25");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=899.28,582.86");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=924.65,669.38");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=927.27,746.98");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1005.80,816.47");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1005.54,898.30");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=946.11,879.80");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=902.78,817.19");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=857.62,977.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=920.86,967.04");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=986.09,986.43");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=765.04,988.92");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=718.26,918.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=793.68,878.07");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1091.23,842.69");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1159.13,911.71");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1235.12,852.59");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1182.36,800.21");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1211.54,742.41");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1276.93,803.76");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1352.75,841.80");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1335.52,758.51");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1456.29,794.62");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1464.28,854.58");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1432.54,922.17");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1349.88,961.07");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1314.38,899.71");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1244.90,942.55");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1471.66,691.35");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1378.46,715.15");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1528.98,637.20");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1606.30,608.97");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1655.14,670.05");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1691.62,594.99");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1738.19,522.20");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1807.89,581.84");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1768.60,627.83");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1674.68,525.40");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1593.42,485.38");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1500.61,515.37");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1435.93,483.54");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1373.21,486.94");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1408.38,389.45");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1339.61,400.35");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1280.98,319.49");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1210.15,344.75");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1143.60,375.28");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1068.53,402.08");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1037.24,292.37");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1355.19,289.46");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1392.89,217.19");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1459.27,251.70");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1229.52,244.05");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1280.69,205.61");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1341.32,169.40");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1281.99,119.20");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1225.35,166.53");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1174.59,100.49");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1422.00,155.30");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1559.44,189.03");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1523.17,123.13");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1526.11,36.74");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1588.14,65.71");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1636.98,107.17");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1684.94,33.92");
    }

}

