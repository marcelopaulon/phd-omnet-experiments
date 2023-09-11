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

network DADCA_ACK_n100_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor;p=33.78,45.80");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=6.68,138.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=122.40,32.79");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=101.54,172.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=151.99,136.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=16.38,264.27");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=185.28,83.96");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=235.15,134.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=243.85,216.77");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=304.06,218.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=285.84,310.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=332.06,352.80");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=253.03,371.07");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=372.34,406.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=335.30,485.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=272.17,479.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=310.54,569.46");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=390.61,521.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=445.62,493.52");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=466.06,416.23");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=205.43,471.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=249.87,564.61");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=183.88,341.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=417.58,233.01");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=440.27,326.25");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=520.21,302.54");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=556.50,366.75");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=630.76,354.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=653.80,428.57");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=588.46,308.37");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=635.24,245.02");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=668.83,180.65");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=380.42,286.39");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=457.08,627.31");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=503.19,578.48");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=554.89,433.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=605.59,592.99");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=669.89,543.21");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=727.64,581.40");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=548.75,626.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=631.00,663.10");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=680.89,747.71");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=590.31,769.14");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=555.04,850.47");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=512.46,789.10");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=459.23,832.70");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=667.31,835.46");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=661.86,903.14");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=625.72,973.49");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=762.75,917.50");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=853.08,938.55");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=820.40,854.15");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=881.55,799.95");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=947.58,891.05");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=941.49,804.82");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=921.09,735.61");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=880.51,646.09");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=934.38,606.89");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1016.22,627.09");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=977.73,701.19");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=831.71,737.14");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1041.79,688.11");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1115.34,686.32");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1077.78,596.51");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1153.71,570.18");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1122.57,486.91");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1047.59,435.59");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1053.21,513.51");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1184.33,498.48");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1244.81,518.89");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1251.37,592.16");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1306.70,635.78");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1317.87,562.14");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1274.59,717.43");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1288.81,786.59");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1389.34,690.24");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1425.40,613.09");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1463.44,562.09");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1502.58,637.51");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1575.97,658.54");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1622.32,614.15");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1583.58,552.52");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1511.73,485.62");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1574.50,431.70");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1655.46,390.53");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1673.99,459.13");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1753.04,518.76");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1665.40,542.58");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1638.34,678.60");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1458.04,688.35");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1507.62,762.48");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1578.31,361.26");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1500.14,385.68");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1421.87,390.99");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1384.41,340.53");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1375.53,436.52");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1446.73,451.00");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1401.50,500.85");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1257.93,425.89");
    }

}

