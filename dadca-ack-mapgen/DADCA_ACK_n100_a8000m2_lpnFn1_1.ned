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

network DADCA_ACK_n100_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor;p=33.77,159.02");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=81.96,209.47");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=26.22,269.64");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=19.84,340.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=96.26,357.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=143.26,301.94");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=166.71,64.84");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=230.98,49.29");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=62.74,46.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=305.32,119.12");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=349.62,160.15");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=411.55,187.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=368.99,238.38");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=478.20,256.45");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=522.46,321.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=472.15,371.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=569.84,425.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=660.57,436.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=600.56,496.98");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=686.68,499.66");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=642.27,584.03");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=721.23,575.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=767.11,489.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=842.89,429.66");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=856.08,489.47");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=925.87,523.55");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=890.47,575.04");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=828.62,611.55");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=816.05,688.24");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=911.25,454.75");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=939.53,384.01");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1028.78,424.84");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1073.15,511.42");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1111.77,589.50");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1044.82,617.26");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1021.07,556.56");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=941.30,632.77");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=949.77,698.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1089.41,420.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1159.60,427.70");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1238.17,335.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1306.49,302.49");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1259.38,249.66");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1260.56,184.05");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1251.65,104.21");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1205.28,37.39");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1320.22,141.84");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1361.61,220.15");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1430.37,175.81");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1445.58,270.97");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1534.67,248.04");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1545.25,325.08");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1474.46,350.69");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1407.83,411.94");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1518.91,419.54");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1442.35,477.05");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1411.62,544.93");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1486.29,541.11");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1552.76,606.39");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1632.20,598.31");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1565.14,473.57");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1319.85,543.76");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1279.63,488.46");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1361.72,472.43");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1398.04,647.76");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1315.23,609.98");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1255.31,674.62");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1330.26,678.86");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1258.00,586.07");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1180.42,545.17");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=991.68,291.96");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=941.56,217.64");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=846.72,189.60");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=836.78,115.80");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=831.72,272.08");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=861.56,332.06");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=771.94,182.85");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=895.83,91.24");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=770.10,113.01");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=701.56,91.98");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=607.92,97.82");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=732.92,288.66");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=644.95,289.80");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=682.88,227.69");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=599.59,201.57");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=505.11,178.13");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=279.76,273.38");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=320.12,322.20");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=402.64,362.54");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=338.40,390.80");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=367.40,473.32");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=460.73,439.56");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=511.83,487.62");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=730.59,687.84");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=687.76,758.06");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=660.04,827.35");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=577.20,799.43");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=502.88,861.51");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=427.58,878.98");
    }

}

