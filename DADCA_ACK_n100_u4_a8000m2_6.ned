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

network DADCA_ACK_n100_u4_a8000m2_6
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
        @display("i=misc/sensor;p=415.89,7931.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=796.24,7829.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1305.31,7928.64");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1728.31,7793.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2196.57,7943.62");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3876.42,7816.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4535.93,7846.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5055.60,7884.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1522.27,7505.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2876.13,7734.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3516.22,7629.79");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5542.28,7751.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=307.78,7367.79");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=680.40,7439.35");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1066.74,7392.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2152.83,7307.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2982.23,7349.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3615.95,7270.64");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4044.95,7289.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4450.36,7471.60");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5206.60,7504.27");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2519.91,7212.51");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4800.64,7049.62");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5384.28,7035.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5741.30,7130.08");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=343.08,6935.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1067.18,6926.64");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1718.69,6920.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2973.45,6816.17");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3637.32,6914.80");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4214.13,6849.24");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2092.88,6672.17");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2579.50,6723.17");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3325.52,6751.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4531.55,6689.26");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4933.17,6563.20");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=94.18,6328.66");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=766.64,6409.68");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1119.30,6325.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1467.70,6391.66");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3069.48,6438.76");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3692.98,6492.13");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4581.46,6335.25");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5506.35,6476.18");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2009.42,6124.52");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2457.92,6265.56");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4093.91,6211.98");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5212.45,6252.98");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=480.15,5869.64");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1169.55,5848.68");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1662.25,6006.56");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2257.31,5869.79");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2822.01,5835.46");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3220.75,5814.29");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4246.94,5851.11");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5501.81,5874.80");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1584.65,5581.03");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3807.56,5635.01");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4853.83,5663.13");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5960.77,5739.54");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=128.82,5350.61");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=499.31,5486.38");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=955.90,5503.75");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2170.17,5475.59");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2782.85,5430.83");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3263.46,5344.37");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4276.86,5469.87");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5932.63,5380.67");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=583.50,5119.03");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1126.04,5060.44");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1757.75,5094.34");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2972.89,5045.53");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3796.09,5143.49");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4799.97,5266.63");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5359.23,5197.12");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2610.12,4999.63");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3357.98,4975.95");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4308.59,4929.97");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5025.86,4816.99");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5510.35,4826.72");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=451.81,4617.01");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1073.30,4708.55");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1758.92,4584.72");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2193.01,4700.71");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2791.13,4621.27");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3387.30,4587.79");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4468.17,4553.58");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3110.77,4295.73");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3822.57,4349.91");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5032.49,4459.23");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5818.91,4428.14");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=103.81,4099.28");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1136.17,4260.49");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1816.54,4175.32");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2183.05,4229.63");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5401.12,4280.25");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=536.87,3924.41");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2493.91,3988.71");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2946.61,3917.43");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3414.28,3919.66");
    }

}

