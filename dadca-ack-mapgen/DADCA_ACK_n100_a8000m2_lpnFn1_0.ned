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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=31.88,70.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=100.56,179.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=47.05,234.13");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=112.37,250.09");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=97.14,311.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=231.48,255.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=182.00,317.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=51.91,370.30");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=119.52,380.65");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=167.58,421.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=261.75,410.42");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=304.13,471.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=340.18,383.51");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=395.02,326.25");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=464.27,257.99");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=544.90,213.09");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=535.00,300.75");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=607.32,238.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=690.87,207.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=643.65,152.18");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=758.90,232.92");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=762.69,306.47");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=657.75,309.60");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=590.96,339.41");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=565.60,413.03");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=598.20,484.42");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=649.37,523.16");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=624.92,605.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=572.24,539.60");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=474.30,559.42");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=473.25,633.48");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=464.88,728.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=400.55,791.61");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=415.26,857.05");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=277.20,797.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=332.74,740.35");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=379.73,700.72");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=381.01,548.70");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=395.07,614.09");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=316.22,570.10");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=382.93,442.15");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=441.74,470.70");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=664.69,450.69");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=348.06,199.75");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=414.39,176.56");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=278.89,298.84");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=339.87,294.95");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=442.89,392.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=208.59,490.58");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=135.27,541.70");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=147.53,603.63");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=113.87,664.79");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=43.91,627.06");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=58.36,777.81");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=66.90,852.19");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=128.48,802.56");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=74.34,923.53");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=140.22,901.35");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=158.47,991.50");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=123.75,1051.13");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=181.22,851.80");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=286.41,681.11");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=264.47,620.01");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=476.88,876.35");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=346.32,958.26");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=326.72,1055.81");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=269.50,1079.04");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=277.44,1175.44");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=343.47,1247.52");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=427.00,1237.75");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=328.04,1348.29");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=275.72,1282.47");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=231.93,1332.38");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=172.09,1379.80");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=100.83,1334.09");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=19.71,1370.28");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=16.91,1447.60");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=108.73,1427.82");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=160.41,1458.63");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=107.89,1198.51");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=150.46,1284.24");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=243.21,1525.94");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=328.43,1539.60");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=320.04,1620.34");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=401.85,1600.72");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=464.97,1627.25");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=560.58,1646.92");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=578.22,1710.69");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=641.69,1647.33");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=696.31,1591.38");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=669.87,1509.72");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=640.21,1454.26");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=618.75,1397.07");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=707.21,1439.10");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=625.19,1759.10");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=623.83,1833.54");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=595.00,1586.60");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=568.82,1454.37");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=581.50,1515.14");
    }

}

