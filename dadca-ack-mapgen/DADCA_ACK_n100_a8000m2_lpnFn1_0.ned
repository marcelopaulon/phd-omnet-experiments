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

network DADCA_ACK_n100_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=31.66,141.16");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=79.44,214.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=133.65,244.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=119.88,314.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=111.89,390.76");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=128.40,458.82");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=209.05,443.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=304.56,420.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=339.23,328.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=403.60,359.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=485.73,399.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=549.88,362.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=494.78,328.63");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=476.65,237.41");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=400.28,465.62");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=444.93,525.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=350.81,540.10");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=289.11,516.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=276.13,577.42");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=199.36,575.65");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=138.49,654.19");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=78.90,589.52");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=137.06,548.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=45.99,524.74");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=207.27,288.04");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=350.37,220.61");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=425.14,273.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=544.93,429.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=519.43,589.05");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=596.75,631.49");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=620.04,719.93");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=499.46,662.42");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=427.82,679.15");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=422.68,603.66");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=227.88,517.05");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=548.32,493.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=601.24,275.38");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=518.76,98.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=572.46,128.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=587.79,34.69");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=652.91,60.80");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=712.33,83.44");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=794.01,2.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=787.76,91.50");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=752.72,176.87");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=704.46,247.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=702.02,336.00");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=787.23,335.61");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=750.47,397.97");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=725.98,496.86");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=672.99,538.94");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=702.37,633.84");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=752.89,558.69");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=838.78,647.59");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=903.43,708.59");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=927.89,643.69");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=875.73,596.67");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=831.27,507.48");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=650.47,424.99");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=888.91,445.12");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=809.36,439.79");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=775.18,645.47");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=962.13,586.50");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=853.81,767.81");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=944.65,773.51");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=899.26,854.63");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1014.93,808.57");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1050.59,885.75");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1117.35,903.47");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1081.20,980.86");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1009.53,1013.38");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=991.56,952.80");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=902.64,974.66");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=858.83,1030.60");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=921.35,1082.29");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=931.21,1161.34");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=978.23,1108.21");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=974.05,1218.63");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1083.05,1201.85");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1063.26,1119.77");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1104.45,1072.45");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=972.36,866.30");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1100.20,747.79");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1192.53,769.68");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1240.99,829.99");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1252.11,734.89");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1263.06,665.81");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1328.66,677.77");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1375.57,742.62");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1434.13,804.43");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1379.74,873.48");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1366.69,948.63");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1334.11,1003.50");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1245.66,1043.83");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1308.15,1074.57");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1309.05,1168.80");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1223.76,1210.46");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1387.94,1135.23");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1409.36,1042.45");
    }

}

