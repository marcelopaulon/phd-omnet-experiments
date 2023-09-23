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

network DADCA_ACK_n100_u3_a8000m2_0
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
        @display("i=misc/sensor;p=424.70,7891.51");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2737.22,7930.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3143.22,7885.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3783.18,7792.24");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4323.78,7805.44");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4962.73,7899.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5485.85,7959.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5834.26,7871.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=910.32,7541.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5244.04,7525.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=428.58,7398.20");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1689.88,7441.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2193.29,7448.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2773.56,7450.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3652.77,7405.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4476.43,7426.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5759.40,7314.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=692.71,7016.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1377.06,7243.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4090.22,7240.00");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1751.12,6991.15");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2247.22,6793.81");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2588.56,6922.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3242.05,7001.02");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3654.81,6865.90");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4592.61,6825.75");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5144.62,6995.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5831.83,6964.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1236.99,6745.85");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1604.83,6575.53");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1984.76,6521.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4071.25,6554.95");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5148.73,6520.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=620.92,6444.70");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1026.13,6438.94");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2849.54,6465.67");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3771.38,6298.77");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4716.99,6323.33");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5662.55,6409.83");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=515.44,6030.20");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1264.41,6075.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1740.32,6077.15");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2455.43,6217.09");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4067.83,6109.51");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=912.36,5884.07");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2618.10,5874.37");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3753.80,5850.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4532.76,5928.33");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5211.38,5894.61");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5631.64,5841.05");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=259.57,5658.17");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=625.29,5633.70");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1662.56,5664.35");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2210.15,5662.22");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3137.47,5721.37");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4161.32,5757.37");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=147.82,5278.74");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=845.57,5330.36");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1278.88,5491.82");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1734.25,5320.58");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2592.95,5365.60");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3689.00,5418.85");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4696.70,5463.88");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5248.25,5504.06");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5748.50,5399.59");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=515.18,5135.82");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2145.48,5259.96");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3182.49,5055.86");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4141.31,5087.80");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4799.83,5057.33");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1031.74,4921.64");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1547.51,5003.18");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2237.04,4911.99");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2613.70,4804.50");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3513.40,4908.02");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5691.67,4829.56");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1.60,4707.74");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=464.42,4573.62");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1647.72,4653.76");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4143.25,4675.12");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1144.84,4489.33");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2214.28,4338.43");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3047.73,4414.49");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4739.96,4517.68");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5396.87,4293.67");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=79.02,4224.24");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1564.12,4190.29");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2658.20,4085.39");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3730.97,4279.93");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5988.85,4153.46");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=569.23,3892.20");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=963.71,3871.56");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1566.29,3795.42");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2031.78,3813.38");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2961.59,3879.91");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3293.43,3991.59");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3654.28,3872.51");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4015.35,3951.05");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4910.27,3905.57");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=25.38,3771.72");
    }

}

