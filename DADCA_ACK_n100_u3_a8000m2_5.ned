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

network DADCA_ACK_n100_u3_a8000m2_5
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
        @display("i=misc/sensor;p=737.46,7874.38");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1390.92,7926.31");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3058.99,7847.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4015.86,7932.68");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4644.05,7787.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5599.07,7997.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=62.70,7693.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3693.15,7592.83");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4334.95,7584.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5142.62,7710.71");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=116.91,7337.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=457.52,7465.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3136.55,7295.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4854.23,7479.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5187.84,7357.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5891.82,7503.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=977.43,7114.13");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1350.50,7247.82");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2710.16,7011.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4375.83,7236.16");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4910.63,7039.00");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=158.11,6897.01");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=572.14,6990.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3591.76,6943.31");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4111.20,6973.46");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4424.69,6812.80");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5750.51,6992.52");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=949.38,6555.32");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1319.38,6681.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2093.90,6655.92");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2473.34,6590.90");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2928.25,6526.65");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3354.23,6626.84");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4653.56,6517.91");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5232.97,6754.74");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5943.81,6697.18");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=365.57,6358.78");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2110.43,6304.75");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4026.27,6400.97");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4975.46,6375.64");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=647.54,6086.19");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1134.24,6139.71");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1721.39,6243.33");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3746.70,6153.63");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4403.96,6241.77");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5247.86,6090.03");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5675.58,6214.77");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=7.17,5875.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=988.23,5788.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1612.02,5883.13");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2035.96,5895.51");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2687.68,5937.02");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3266.15,5985.11");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4328.09,5811.52");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4740.16,5980.69");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5935.59,5966.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=220.11,5567.34");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=669.18,5620.59");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3050.10,5688.51");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3439.55,5547.66");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4873.38,5574.85");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5514.08,5754.59");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5997.85,5608.22");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1225.93,5463.96");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1803.55,5451.71");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4043.66,5377.69");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5449.35,5399.35");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=742.52,5206.93");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2725.93,5183.48");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3461.84,5171.69");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4621.02,5156.07");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5930.38,5088.83");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=252.64,5007.14");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1322.35,4833.46");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3087.76,4961.89");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4955.57,4907.15");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=782.35,4635.90");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2131.19,4712.01");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2718.56,4615.08");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3574.29,4768.64");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4318.16,4765.46");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5943.36,4669.46");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=178.25,4488.81");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1682.88,4297.08");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3254.33,4517.81");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3864.83,4533.17");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4644.87,4528.00");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4985.90,4304.49");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5367.13,4382.00");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=457.76,4259.43");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=810.47,4148.59");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1325.85,4040.57");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2015.91,4151.75");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2430.49,4186.21");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3029.89,4077.40");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4350.10,4099.07");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5835.20,4274.58");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=33.84,4040.06");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=514.77,3897.92");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1853.58,3796.24");
    }

}

