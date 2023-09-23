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

network DADCA_ACK_n100_u4_a8000m2_0
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
        @display("i=misc/sensor;p=95.49,7868.57");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=954.45,7795.80");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1714.95,7752.55");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2504.99,7924.24");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5232.55,7964.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5753.47,7756.75");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2117.86,7603.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2810.83,7715.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3165.68,7713.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4357.88,7648.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=34.94,7274.57");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=623.28,7427.53");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1322.12,7449.71");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1923.85,7291.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3797.57,7467.68");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4836.20,7495.66");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5187.59,7326.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5766.13,7333.44");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=531.37,7010.78");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2157.07,7017.93");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2468.68,7252.83");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2821.79,7153.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3290.08,7047.62");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4151.98,7054.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4641.95,7060.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=992.89,6822.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1678.95,6999.94");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3615.10,6877.21");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5414.46,6987.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=169.71,6740.16");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2476.97,6634.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2890.40,6748.34");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4248.87,6711.37");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5176.38,6683.09");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5724.72,6703.55");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=607.48,6435.74");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1256.95,6304.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2022.72,6499.59");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2826.84,6353.46");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3938.91,6441.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4525.48,6482.87");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5159.05,6296.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3157.66,6220.42");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3674.28,6206.76");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4101.60,6129.28");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4744.34,6197.06");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5792.80,6227.46");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=283.10,5930.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=771.99,5884.66");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1140.26,5832.52");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1613.45,5785.51");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2757.72,5796.28");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3561.02,5850.84");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4548.85,5888.03");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5411.16,5889.05");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3178.21,5659.03");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4029.80,5720.65");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4889.91,5735.90");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5873.66,5581.10");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=232.92,5285.09");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=597.36,5285.13");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1895.96,5280.13");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2376.77,5468.30");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3470.91,5418.68");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4271.74,5435.65");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4935.38,5326.54");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5243.75,5509.81");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1215.02,5250.42");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2324.00,5092.32");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3046.38,5224.97");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3799.71,5255.50");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4676.91,5037.49");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5778.21,5061.02");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=56.07,4887.77");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=777.29,4912.20");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1212.02,4861.02");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1722.44,4975.28");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2672.06,4922.23");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3259.67,4857.19");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3662.47,4906.24");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=431.21,4619.05");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4265.72,4678.93");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5210.38,4635.80");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1225.26,4490.52");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2130.16,4458.52");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4741.69,4502.52");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5829.17,4493.26");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=253.11,4122.26");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1012.47,4146.20");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1739.81,4206.98");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2540.14,4154.01");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3186.36,4250.99");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3711.06,4259.06");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4125.05,4058.22");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4633.96,4140.62");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5840.95,4089.76");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=595.18,4038.89");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2267.76,3919.39");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5159.39,3979.37");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=279.97,3713.52");
    }

}

