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
        @display("i=misc/sensor,p=86.33,175.92");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=181.21,53.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=181.61,159.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=238.80,134.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=274.38,209.38");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=246.39,299.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=283.78,379.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=245.59,438.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=167.64,437.65");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=109.83,416.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=107.30,356.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=186.30,302.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=99.88,280.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=152.70,540.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=202.88,578.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=235.54,645.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=292.58,669.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=348.29,617.03");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=417.91,615.95");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=452.75,544.02");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=448.72,703.24");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=459.77,769.34");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=531.32,798.39");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=553.83,655.31");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=577.90,738.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=668.19,700.44");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=605.92,622.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=479.59,601.38");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=359.34,686.38");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=328.84,547.95");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=372.24,467.26");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=312.95,449.50");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=563.58,539.99");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=164.16,669.95");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=216.50,750.91");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=208.91,814.14");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=136.32,858.83");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=281.77,851.04");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=346.77,862.57");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=187.13,892.65");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=136.44,962.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=199.02,971.52");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=261.67,994.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=210.23,1033.28");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=53.49,802.73");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=83.86,723.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=61.43,666.31");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=60.31,603.02");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=57.52,526.09");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=6.98,754.47");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=156.66,247.92");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=16.58,373.02");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=33.59,308.54");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=113.55,487.31");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=266.94,509.49");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=269.54,791.50");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=377.22,743.87");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=323.89,938.54");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=374.82,992.60");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=407.66,1063.53");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=380.98,1152.86");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=446.11,1131.35");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=286.54,1083.55");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=207.62,1112.02");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=123.27,1111.46");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=179.84,1178.78");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=245.36,1172.98");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=121.03,1241.46");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=73.48,1185.73");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=22.64,1267.91");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=39.49,1339.42");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=32.81,1422.61");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=12.92,1501.61");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=81.61,1553.86");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=140.07,1575.36");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=174.19,1505.26");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=155.03,1644.65");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=194.65,1690.46");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=207.93,1606.57");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=178.41,1385.91");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=244.14,1378.70");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=177.65,1313.93");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=92.80,1465.51");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=276.74,1564.31");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=250.81,1486.90");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=326.22,1387.13");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=311.81,1499.60");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=115.15,1348.91");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=276.66,1259.12");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=372.72,1265.71");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=314.33,1307.15");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=405.17,1382.97");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=433.48,1313.70");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=356.96,1457.10");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=417.70,1500.94");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=493.64,1442.73");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=152.95,1442.17");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=58.69,1626.49");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=4.21,1598.83");
    }

}

