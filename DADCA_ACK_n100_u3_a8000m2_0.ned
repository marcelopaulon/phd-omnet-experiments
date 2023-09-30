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
        @display("i=misc/sensor;p=1729.44,7801.65");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2195.01,7887.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3070.04,7884.51");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4717.73,7856.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5323.57,7882.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=476.82,7654.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2772.01,7568.14");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4404.26,7657.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5018.35,7617.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5820.71,7591.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1234.18,7427.52");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2035.48,7376.08");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2477.10,7343.93");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3100.93,7429.91");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4246.88,7306.55");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5487.46,7365.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=127.04,7182.58");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=487.31,7167.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=826.13,7078.78");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3578.42,7171.13");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4823.35,7196.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1665.74,6950.35");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2292.32,6783.03");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3179.37,6826.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3864.99,6813.24");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5108.87,6883.21");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5688.63,6843.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=6.86,6723.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=393.59,6675.30");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1270.99,6587.89");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3500.75,6550.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4385.42,6629.39");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4828.64,6668.09");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5284.86,6556.34");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=913.73,6458.05");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1966.90,6324.98");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2470.84,6417.74");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3163.81,6345.63");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5723.40,6383.45");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=345.37,6237.01");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=843.39,6023.73");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1462.68,6043.49");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2628.50,6037.68");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3494.86,6137.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3853.47,6248.13");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4256.06,6179.34");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4762.11,6045.01");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5219.01,6118.95");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2091.61,5801.72");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3090.50,5820.05");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4181.56,5778.35");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5714.92,5964.75");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=91.64,5650.43");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=460.37,5766.90");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1163.82,5659.41");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3456.52,5691.85");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3820.01,5606.49");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4871.58,5691.60");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5361.15,5688.38");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=736.47,5379.02");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1713.96,5514.71");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2354.07,5279.74");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2656.20,5469.52");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5596.22,5392.34");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=165.28,5040.18");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1463.58,5076.02");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1994.56,5270.53");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2993.52,5179.14");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3415.52,5271.41");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3893.50,5258.97");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4293.11,5048.39");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4796.78,5203.03");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5184.18,5207.53");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5908.45,5044.89");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=603.64,4925.95");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1019.34,4958.47");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1918.57,4909.14");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2799.81,4791.47");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5193.53,4840.37");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5641.15,4784.18");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=337.32,4546.18");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=722.96,4579.94");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1668.80,4546.60");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2305.89,4693.74");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3457.01,4741.04");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4121.02,4681.54");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4542.04,4597.76");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4892.66,4600.95");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1213.80,4400.37");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2537.32,4403.57");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3811.60,4491.96");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5321.83,4490.25");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=166.30,4202.96");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2076.55,4264.63");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2924.79,4243.27");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3365.85,4259.42");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4426.82,4235.54");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5000.66,4214.01");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5913.83,4138.64");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=697.23,3823.66");
    }

}

