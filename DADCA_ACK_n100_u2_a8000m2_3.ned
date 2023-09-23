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

network DADCA_ACK_n100_u2_a8000m2_3
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
        @display("i=misc/sensor;p=449.76,7836.29");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1105.98,7858.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2757.90,7884.96");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3400.35,7997.43");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4181.42,7802.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5348.35,7897.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=887.79,7529.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2378.57,7749.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3152.14,7716.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3622.03,7658.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4868.58,7593.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5731.75,7751.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=76.70,7364.07");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1277.44,7315.96");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1845.67,7456.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2233.74,7414.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3252.10,7311.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3772.90,7303.32");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4188.71,7284.89");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5577.01,7351.97");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=661.19,7044.83");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2058.41,7074.54");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2426.16,7117.76");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4614.97,7053.35");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1603.64,6898.76");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2793.31,6968.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3496.09,6944.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3999.24,6839.26");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5114.31,7009.62");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5714.52,6981.62");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=439.10,6724.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1046.67,6680.34");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2241.31,6526.52");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2980.15,6643.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4421.43,6728.49");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5554.11,6571.28");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=283.06,6318.13");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1561.84,6385.84");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2583.06,6372.96");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3746.86,6511.52");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4328.35,6352.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4682.63,6467.84");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5100.88,6342.85");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=736.22,6067.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1147.98,6091.91");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2808.35,6056.82");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3930.28,6063.13");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4892.42,6052.02");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5923.57,6074.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=240.61,5797.85");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1445.19,5781.39");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1922.28,5915.30");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2277.26,5882.23");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3456.67,5969.04");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5504.54,5906.56");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=839.15,5671.10");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2661.85,5536.92");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3954.50,5638.71");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4517.74,5737.48");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5154.49,5771.54");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5589.85,5540.13");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1820.77,5399.80");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2234.32,5353.89");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3097.34,5444.82");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4563.91,5359.33");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4981.34,5302.77");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=476.42,5142.77");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1144.97,5147.46");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2786.45,5061.68");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3349.47,5071.96");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3651.61,5264.23");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4140.18,5170.57");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5666.86,5173.51");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1886.46,4912.05");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4656.59,5008.49");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5136.29,4813.19");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=437.59,4548.52");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=872.54,4592.21");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1414.78,4717.97");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2398.73,4667.40");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3074.12,4738.05");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3566.39,4695.44");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4204.99,4587.75");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4557.38,4596.27");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=94.18,4379.60");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1797.86,4362.55");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3937.65,4302.97");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4894.53,4440.87");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5566.94,4356.18");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=554.29,4094.53");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=932.63,4083.53");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1495.96,4168.70");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2626.27,4190.92");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3532.27,4239.25");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2149.73,3971.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2783.28,3860.05");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3258.31,3797.25");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4032.62,3821.98");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4836.85,4012.62");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5419.16,3987.57");
    }

}

