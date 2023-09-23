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

network DADCA_ACK_n100_u16_a8000m2_1
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
        @display("i=misc/sensor;p=577.32,7912.40");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1354.85,7828.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1703.48,7795.27");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2623.82,7804.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3441.01,7790.57");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4662.18,7925.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5739.93,7943.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=957.09,7600.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3003.79,7687.59");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4431.98,7610.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5298.64,7743.68");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=183.64,7478.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1610.78,7452.93");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2251.84,7406.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2940.24,7305.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3795.85,7431.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5703.00,7470.27");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1244.45,7179.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1885.84,7141.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3381.69,7143.16");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3973.04,7087.39");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4421.32,7054.56");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4933.84,7174.00");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=46.80,6877.40");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=644.31,6984.31");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1962.22,6781.55");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2483.50,6795.71");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3704.16,6850.51");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5270.95,6919.35");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5925.99,6970.99");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1596.24,6586.58");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2941.73,6726.38");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4099.73,6701.84");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4456.94,6572.39");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5.27,6280.39");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1139.90,6283.91");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2377.74,6429.30");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3109.08,6376.00");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3504.00,6490.22");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4737.13,6288.48");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5062.01,6486.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5583.83,6509.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5925.91,6391.49");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=730.40,6036.16");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1495.63,6179.62");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1922.27,6203.14");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2848.84,6119.12");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3687.18,6173.84");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4114.70,6169.16");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5484.58,6147.76");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=382.62,5994.38");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1672.51,5860.12");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2145.48,5832.69");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2507.84,5993.37");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3508.35,5860.48");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5024.16,5930.02");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5657.33,5830.14");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=822.79,5683.22");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1201.27,5652.55");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3158.02,5630.47");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4121.23,5717.05");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4587.85,5763.17");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5240.71,5623.00");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=289.02,5338.15");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1714.74,5340.69");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2580.25,5507.46");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3649.71,5506.69");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4445.68,5352.49");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=566.72,5109.61");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1045.40,5122.16");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2211.61,5276.11");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2490.08,5046.69");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2853.09,5047.47");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3298.65,5032.04");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4883.86,5235.25");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5431.54,5242.35");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5914.80,5263.92");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1866.58,5008.06");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2194.21,4836.04");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3897.76,4889.09");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4787.65,4890.28");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5895.25,4885.21");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=433.86,4753.74");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=786.49,4606.60");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1278.58,4759.08");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1668.53,4565.69");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2744.21,4712.90");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3716.78,4570.32");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4258.44,4622.26");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5563.87,4698.36");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=130.98,4476.57");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2544.53,4395.08");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3359.02,4436.21");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5051.39,4461.20");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5990.71,4401.10");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1139.30,4109.27");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2884.75,4118.48");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3478.81,4095.77");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3925.36,4275.89");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4508.79,4215.95");
    }

}

