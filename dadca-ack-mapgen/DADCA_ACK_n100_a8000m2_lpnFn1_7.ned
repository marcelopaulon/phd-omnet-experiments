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

network DADCA_ACK_n100_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=127.11,31.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=213.64,4.69");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=244.87,98.79");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=318.58,111.67");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=306.72,202.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=217.99,224.78");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=209.86,159.31");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=171.54,334.04");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=249.67,322.28");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=134.32,224.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=65.65,308.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=68.80,381.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=38.00,469.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=122.18,428.40");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=35.97,556.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=68.55,644.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=143.85,557.91");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=12.32,716.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=96.02,701.41");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=150.04,638.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=168.03,730.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=228.69,710.48");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=327.20,695.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=359.81,615.24");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=337.34,797.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=406.23,840.74");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=313.55,862.88");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=255.24,799.46");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=315.62,930.40");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=388.35,938.44");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=271.98,1064.64");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=364.08,1034.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=210.40,852.61");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=174.45,802.36");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=158.93,890.96");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=228.44,916.53");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=124.19,957.64");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=75.30,918.33");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=83.41,852.44");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=32.13,792.55");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=226.84,635.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=294.47,575.32");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=388.65,521.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=477.46,637.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=546.63,584.76");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=569.65,488.37");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=549.58,656.58");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=535.40,741.34");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=628.96,784.34");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=667.49,830.81");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=646.79,905.67");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=643.49,997.90");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=681.78,1065.20");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=540.37,849.94");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=450.23,714.73");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=465.19,555.41");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=398.67,423.15");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=370.96,742.30");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=531.80,938.66");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=432.02,1007.78");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=266.41,971.62");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=443.33,906.70");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=682.06,755.90");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=672.35,670.39");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=743.67,750.09");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=790.11,826.44");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=850.38,901.77");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=891.49,945.60");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=908.06,884.81");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=870.06,1040.87");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=792.38,1081.14");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=766.46,1014.92");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=799.86,941.04");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=898.41,811.05");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=974.28,793.10");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=882.92,724.21");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=894.52,627.56");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1036.04,804.64");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1058.81,889.35");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=987.20,854.79");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=816.42,750.29");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=736.57,897.93");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=602.71,953.04");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=387.26,1104.80");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=391.05,1172.44");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=468.26,1174.45");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=493.25,1233.43");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=555.78,1255.48");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=503.08,1332.38");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=617.51,1301.62");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=712.43,1317.98");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=729.99,1404.96");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=688.18,1467.58");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=617.03,1524.73");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=660.61,1591.96");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=592.21,1656.31");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=514.87,1678.67");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=447.77,1692.66");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=380.71,1761.68");
    }

}

