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

network DADCA_ACK_n100_u16_a8000m2_2
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
        @display("i=misc/sensor;p=745.40,7799.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1194.99,7949.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2404.79,7931.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4147.22,7941.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5981.73,7864.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1048.15,7530.96");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1738.25,7727.94");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2247.05,7507.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2882.61,7625.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3565.15,7575.62");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3979.25,7582.86");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4598.21,7516.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5770.70,7539.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=72.77,7448.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1924.55,7284.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=547.41,7199.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1555.08,7232.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2540.62,7225.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3170.76,7247.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3934.05,7215.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4542.91,7170.11");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5006.51,7141.91");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5465.17,7106.35");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=217.49,6986.25");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1083.04,6865.36");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1512.61,6766.60");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1863.19,6888.04");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2212.78,6952.34");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3760.58,6781.33");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5935.49,6933.11");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=345.21,6626.13");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2455.09,6561.00");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2978.68,6720.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4354.57,6694.40");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4901.88,6680.57");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5702.18,6626.83");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=737.94,6484.50");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1197.51,6384.67");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1859.70,6282.45");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2966.17,6308.66");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3448.94,6471.58");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3931.78,6366.64");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4598.42,6436.93");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=371.79,6249.00");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=797.01,6092.88");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3647.66,6152.61");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5161.65,6236.23");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5783.19,6248.11");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=403.07,5866.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1395.88,5901.32");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2368.40,5992.11");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3249.62,5849.40");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4151.84,5905.60");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4971.00,5893.95");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5608.81,5778.31");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2153.88,5669.58");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2840.60,5587.64");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4535.67,5650.56");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=24.65,5359.52");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=650.17,5495.38");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1070.59,5295.10");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1817.65,5423.29");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2580.33,5279.59");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3303.09,5396.60");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3667.83,5456.02");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4287.02,5290.94");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5376.24,5372.36");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4688.23,5099.63");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5750.84,5114.19");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=126.10,4960.95");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=512.30,4866.85");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1510.16,4939.29");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2081.21,4964.49");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2820.99,4962.92");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3338.52,4906.25");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3760.58,4955.84");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=898.00,4635.55");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1877.38,4640.26");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2414.32,4592.11");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3742.40,4591.41");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4308.65,4638.15");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5688.99,4757.76");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=321.29,4488.89");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=632.23,4301.78");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1188.40,4374.14");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2149.24,4302.79");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2933.66,4443.34");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4824.44,4336.10");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=241.83,4108.43");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1523.10,4256.13");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2662.42,4121.27");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3281.68,4167.51");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4409.43,4209.15");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5505.49,4214.20");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5945.66,4258.32");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=799.71,3988.40");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1838.07,3977.93");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2286.99,3941.96");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3798.96,3894.93");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5877.00,3810.03");
    }

}

