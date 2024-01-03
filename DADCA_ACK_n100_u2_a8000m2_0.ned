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

network DADCA_ACK_n100_u2_a8000m2_0
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
        @display("i=misc/sensor;p=362.11,7758.97");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=840.30,7950.65");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1353.34,7787.72");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2461.06,7925.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3098.86,7979.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4443.20,7857.77");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5706.66,7770.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1009.85,7625.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1805.58,7717.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2301.47,7598.05");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2699.45,7505.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3436.95,7541.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4085.35,7741.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4877.65,7661.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5265.78,7529.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=90.50,7341.34");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3077.07,7388.58");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4468.70,7278.50");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1116.42,7146.56");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1527.57,7166.87");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2144.63,7247.30");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2521.20,7153.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2852.54,7012.63");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3907.20,7248.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5447.27,7116.65");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=330.16,6862.91");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2170.62,6809.49");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3928.67,6846.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4274.15,6767.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4967.35,6873.26");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5884.51,6993.03");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=894.68,6637.56");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1432.41,6691.03");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2600.98,6638.75");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3046.07,6571.48");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3379.36,6757.70");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4641.71,6613.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5533.95,6589.28");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=32.89,6397.46");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=475.19,6512.62");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1952.03,6345.36");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2330.96,6364.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3671.86,6510.58");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=328.16,6154.08");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=786.80,6066.72");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1171.89,6258.45");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1527.04,6261.70");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2845.26,6081.71");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3304.70,6091.97");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4269.81,6025.17");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4592.32,6195.57");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4979.59,6202.15");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5530.76,6130.32");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1460.45,5832.39");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2492.33,6014.91");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3669.20,6005.22");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3964.12,5796.13");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4853.72,5825.18");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5786.82,5832.07");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=350.70,5573.14");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=815.65,5707.72");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1876.20,5673.99");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2612.01,5612.02");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3094.75,5615.68");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4403.42,5696.76");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5195.08,5633.97");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2381.31,5323.16");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3485.96,5510.64");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5066.61,5307.46");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5430.78,5338.72");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1022.06,5193.71");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1418.39,5237.56");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4062.38,5223.34");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4676.73,5268.01");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5812.68,5150.73");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=85.94,5022.32");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=720.29,5007.42");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2873.75,5010.21");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3295.09,4987.61");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3732.42,4809.79");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4271.74,4872.44");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4714.33,4860.75");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5427.05,4867.02");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=298.31,4726.94");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=652.71,4598.32");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1210.55,4765.34");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2450.61,4731.29");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2746.31,4536.38");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3141.24,4564.84");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5081.85,4747.84");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=37.37,4417.19");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1205.68,4322.79");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5076.87,4298.63");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5687.34,4505.59");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1692.07,4259.23");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2130.13,4219.62");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3086.77,4194.79");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3503.51,4084.01");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4070.75,4139.52");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4675.03,4243.53");
    }

}

