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

network DADCA_ACK_n100_u2_a8000m2_1
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
        @display("i=misc/sensor;p=384.32,7780.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4427.50,7795.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4937.05,7776.45");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5412.94,7875.47");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=948.22,7681.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1887.74,7522.17");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2197.22,7734.09");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2738.55,7701.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3266.10,7635.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3969.84,7675.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5178.59,7507.16");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1387.22,7495.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3721.72,7284.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5550.96,7450.82");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5991.62,7346.45");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=122.35,7133.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=784.07,7241.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1783.27,7177.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2138.44,7098.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2929.93,7194.67");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4041.61,7093.78");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4457.10,7243.91");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4902.28,7249.78");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5469.89,7023.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5873.02,7013.96");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1447.00,6993.82");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3407.65,6988.54");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4299.04,6776.99");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5157.23,6764.21");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=568.06,6716.83");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1131.65,6652.15");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2118.94,6582.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2544.53,6530.74");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3740.52,6666.70");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4703.44,6652.09");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=332.47,6411.66");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1072.74,6290.74");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1564.02,6438.47");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3036.00,6455.24");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4311.86,6369.04");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4988.85,6303.77");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=731.37,6191.62");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1866.88,6192.24");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3372.38,6198.96");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3796.97,6219.99");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5317.75,6154.87");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5666.45,6024.16");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1525.12,6019.75");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2704.00,5956.87");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3106.61,5875.25");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3635.67,5907.28");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4653.83,5870.26");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=508.70,5746.42");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=896.91,5737.91");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1615.63,5620.76");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2221.61,5765.97");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2552.79,5537.99");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4330.47,5550.35");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5078.68,5708.48");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5552.07,5581.72");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5900.14,5715.51");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=299.62,5319.25");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1308.83,5430.64");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3605.72,5439.72");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3978.28,5329.68");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=794.81,5170.85");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2007.36,5162.25");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2546.00,5098.51");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3094.52,5177.06");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4175.83,5037.78");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4841.56,5246.68");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5517.97,5190.09");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=986.67,4826.34");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1706.60,4863.62");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3490.97,4952.14");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4666.53,4912.04");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5404.67,4829.77");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5897.05,4992.80");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=383.32,4735.79");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=718.40,4588.88");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1356.48,4767.47");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2125.31,4561.24");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2532.37,4550.28");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3698.42,4549.51");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4453.70,4608.63");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5046.66,4655.23");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1094.98,4455.75");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3087.03,4454.18");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4196.86,4333.89");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4992.35,4302.58");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5905.21,4355.16");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=632.10,4230.30");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1601.03,4215.77");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2224.40,4094.16");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4616.83,4221.57");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5493.70,4219.35");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1050.02,3905.39");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2649.96,4034.68");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2941.78,3839.83");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3520.62,3897.69");
    }

}

