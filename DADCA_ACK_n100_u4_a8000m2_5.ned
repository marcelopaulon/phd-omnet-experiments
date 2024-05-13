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

network DADCA_ACK_n100_u4_a8000m2_5
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
        @display("i=misc/sensor;p=235.54,7905.20");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=593.07,7980.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1484.61,7919.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2059.29,7895.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2752.41,7758.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3656.84,7892.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4261.88,7857.69");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4743.74,7912.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5871.54,7949.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=289.77,7558.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=955.06,7666.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1777.98,7589.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2146.99,7536.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3152.52,7704.86");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5357.53,7746.68");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5800.60,7512.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=658.26,7455.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1330.47,7452.44");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2534.37,7422.57");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4336.88,7341.53");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4659.37,7499.84");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1510.77,7126.70");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1945.40,7197.64");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3064.79,7156.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3762.78,7209.17");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4836.40,7184.10");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5194.84,7218.19");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5821.16,7075.48");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=419.78,6816.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=964.89,6964.01");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2501.93,6962.12");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3408.89,6929.43");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4228.71,6846.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4694.28,6839.65");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5426.67,6889.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=998.95,6600.33");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1445.19,6710.99");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1862.15,6749.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2949.17,6585.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3649.61,6651.01");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=344.27,6419.65");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2011.51,6326.94");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2419.18,6372.07");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4702.27,6426.33");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5064.36,6467.54");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5506.15,6457.68");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5983.81,6460.24");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=786.26,6140.47");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1471.02,6226.60");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2944.70,6069.32");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3658.08,6246.14");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4025.77,6217.20");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5842.43,6055.26");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=83.25,6000.92");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1682.41,5842.74");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2356.86,5949.69");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3305.06,5965.75");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4301.17,5918.54");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4816.39,5805.23");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5283.37,5885.47");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=296.19,5532.37");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=668.46,5673.80");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1076.61,5686.49");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2028.21,5719.64");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3007.63,5706.94");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3666.39,5698.70");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4284.46,5564.11");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1266.02,5288.67");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1951.86,5287.39");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2668.04,5519.03");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3321.16,5320.45");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5307.92,5440.77");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5709.80,5441.51");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=733.68,5212.53");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2311.84,5035.81");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2966.41,5077.09");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5355.69,5070.75");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5916.74,5060.83");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=312.84,4883.16");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1552.57,4993.63");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2662.70,4831.92");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3468.99,4948.00");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3883.62,4993.21");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4566.41,4850.43");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4896.99,4993.05");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=933.27,4748.30");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1982.84,4686.02");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2336.21,4663.65");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3170.77,4719.99");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5904.02,4666.91");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=571.33,4327.85");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1391.64,4437.25");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1743.33,4332.68");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2918.01,4469.39");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3833.74,4516.78");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4408.45,4384.49");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4981.68,4315.57");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5420.22,4419.24");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1067.58,4276.59");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1390.63,4084.81");
    }

}

