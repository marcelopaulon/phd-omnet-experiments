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

network DADCA_ACK_n100_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=13.97,64.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=157.35,225.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=210.62,277.62");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=304.99,277.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=307.71,360.82");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=174.82,373.41");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=242.71,337.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=349.52,413.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=412.27,372.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=439.46,292.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=258.88,397.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=410.23,460.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=335.56,519.41");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=286.98,598.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=226.86,610.24");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=210.94,678.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=440.13,521.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=473.43,579.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=542.84,589.83");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=509.24,645.24");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=472.55,734.46");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=625.66,724.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=548.72,786.07");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=674.43,780.59");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=689.69,840.69");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=712.10,715.89");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=552.45,880.71");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=749.68,842.30");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=815.66,797.68");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=836.08,713.02");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=767.45,750.36");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=466.94,907.73");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=486.82,850.02");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=635.19,665.31");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=625.99,585.74");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=526.70,522.75");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=401.82,606.49");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=391.08,691.61");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=324.82,652.09");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=235.64,477.86");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=149.91,562.13");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=114.30,482.20");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=30.66,556.63");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=101.35,611.11");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=141.12,753.99");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=173.75,840.21");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=268.57,856.19");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=325.70,882.14");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=412.03,854.81");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=230.33,920.42");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=166.57,924.68");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=181.25,1009.32");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=199.10,1078.11");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=65.80,1040.85");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=69.94,1123.83");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=129.69,1072.16");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=113.33,985.50");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=6.66,1023.71");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=91.05,1205.62");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=146.54,1258.77");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=127.02,1339.36");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=41.97,1388.11");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=98.32,1456.97");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=156.20,1490.68");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=84.79,1545.68");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=134.84,1609.32");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1.57,1439.36");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=263.35,1533.71");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=242.82,1603.29");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=377.05,1578.74");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=450.57,1548.71");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=415.15,1423.56");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=466.95,1389.87");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=526.91,1360.34");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=606.69,1319.31");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=455.49,1253.48");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=361.05,1357.50");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=308.89,1444.43");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=314.01,1643.04");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=253.24,1664.87");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=327.50,1718.54");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=283.24,1781.02");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=367.77,1488.29");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=322.09,1299.07");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=240.25,1461.61");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=263.25,1354.85");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=264.51,1232.09");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=202.68,1298.13");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=475.13,1493.56");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=487.01,1605.62");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=439.12,1679.78");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=447.53,1747.37");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=386.69,1782.94");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=368.09,1864.82");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=426.12,1892.76");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=511.92,1919.24");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=510.38,1844.88");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=537.92,1761.35");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=630.20,1757.47");
    }

}

