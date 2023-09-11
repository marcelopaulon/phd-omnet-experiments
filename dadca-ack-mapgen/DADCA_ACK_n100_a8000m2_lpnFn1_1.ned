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
        @display("i=misc/sensor;p=25.56,69.10");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=93.08,12.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=85.74,159.22");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=171.80,174.13");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=233.85,161.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=122.39,230.46");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=86.89,294.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=28.84,231.81");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=17.11,318.42");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=96.29,356.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=159.86,316.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=31.07,416.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=88.33,439.87");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=154.68,472.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=202.14,430.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=242.23,479.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=218.09,561.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=156.96,585.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=315.03,602.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=404.97,631.95");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=377.68,715.16");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=449.90,563.64");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=386.46,552.23");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=350.35,656.81");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=252.26,663.18");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=163.79,682.74");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=100.35,630.13");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=23.10,592.21");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=150.73,785.79");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=244.72,769.16");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=205.25,827.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=43.83,751.19");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=28.38,673.25");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=100.84,554.02");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=232.89,348.27");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=316.41,324.63");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=319.58,405.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=231.24,245.11");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=289.29,263.44");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=448.73,286.40");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=376.74,309.21");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=361.74,234.83");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=524.90,230.34");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=477.38,168.82");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=633.63,307.77");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=716.01,308.21");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=665.89,247.56");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=652.52,397.50");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=610.85,474.54");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=529.42,315.08");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=497.13,407.90");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=413.83,447.70");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=272.90,97.57");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=195.17,89.82");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=25.87,129.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=175.95,12.75");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=31.89,6.86");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=408.37,158.54");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=459.14,101.36");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=524.20,124.89");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=515.85,51.58");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=448.90,27.35");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=613.79,116.91");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=645.87,172.38");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=599.48,239.20");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=575.01,361.09");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=713.08,499.51");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=569.83,526.30");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=639.79,530.57");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=710.41,579.29");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=769.15,435.86");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=802.75,357.09");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=779.66,547.77");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=772.01,633.40");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=897.30,433.83");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=856.75,514.52");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=845.52,606.87");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=922.19,326.97");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=953.83,401.68");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=954.44,499.08");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=964.34,566.06");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1016.09,435.24");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1091.11,424.59");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1047.10,367.70");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1177.40,408.10");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1253.58,368.76");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1186.67,324.22");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1349.66,407.78");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1405.50,440.49");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1484.62,466.01");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1529.41,527.42");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1439.97,574.50");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1559.58,428.09");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1613.47,398.51");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1702.56,368.37");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1674.11,428.53");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1769.01,451.16");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1832.29,478.20");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1907.08,466.12");
    }

}

