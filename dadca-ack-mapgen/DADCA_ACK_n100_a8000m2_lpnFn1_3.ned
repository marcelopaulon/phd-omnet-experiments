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

network DADCA_ACK_n100_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor;p=175.63,84.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=150.35,26.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=26.29,60.20");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=143.99,211.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=143.17,305.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=48.66,319.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=77.72,255.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=108.95,409.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=206.11,316.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=263.15,374.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=195.95,387.76");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=162.16,477.60");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=85.49,504.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=127.08,553.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=6.77,409.60");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=17.17,490.62");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=98.22,624.94");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=19.89,597.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=143.53,711.79");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=199.71,644.52");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=328.23,315.44");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=400.72,330.21");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=475.40,364.18");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=461.85,445.25");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=517.57,494.75");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=405.29,569.62");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=477.97,561.52");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=589.05,587.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=661.67,521.21");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=725.55,576.47");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=699.63,449.94");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=784.49,442.58");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=795.52,501.63");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=920.63,453.16");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=913.47,367.56");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=962.50,499.52");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1035.22,514.52");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=795.98,570.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=825.67,664.58");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=883.15,685.75");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=869.42,802.08");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=937.08,764.35");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=988.76,807.71");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=958.71,870.95");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1002.81,946.04");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=959.79,1020.08");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=907.31,978.09");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=901.76,911.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=831.18,946.56");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=786.90,888.51");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=765.59,818.33");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=726.06,753.07");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=655.81,752.84");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=612.72,668.55");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=931.42,624.48");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=997.70,616.85");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1095.70,617.70");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1129.41,681.45");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1079.20,738.92");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1168.99,569.89");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1110.49,554.15");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1240.38,538.68");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1158.25,495.63");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1281.44,462.50");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1341.66,385.48");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1391.25,434.41");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1344.16,313.31");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1248.02,399.28");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1207.82,312.81");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1252.94,235.59");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1351.84,158.33");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1268.59,124.24");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1212.14,187.11");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1267.02,32.66");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1344.32,37.50");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1201.93,4.08");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1133.46,16.17");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1199.01,90.08");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1441.87,155.18");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1507.86,94.04");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1525.92,154.86");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1616.78,114.15");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1676.63,99.96");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1732.90,137.14");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1629.27,11.79");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1701.40,4.12");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1593.84,192.10");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1607.51,272.75");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1583.70,365.37");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1547.74,415.41");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1594.93,468.19");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1590.65,541.93");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1588.19,620.03");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1684.08,522.80");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1689.88,582.78");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1497.83,632.70");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1487.81,704.34");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1548.52,717.42");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1438.72,760.94");
    }

}

