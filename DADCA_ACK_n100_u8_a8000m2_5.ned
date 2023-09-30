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

network DADCA_ACK_n100_u8_a8000m2_5
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
        @display("i=misc/sensor;p=735.97,7947.91");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1259.56,7886.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2990.18,7897.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3700.25,7759.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5288.69,7878.56");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=221.04,7747.28");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1006.48,7627.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1839.75,7667.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2941.08,7514.27");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4749.00,7605.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5579.52,7651.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=428.44,7272.42");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1395.15,7466.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2459.34,7425.49");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3588.11,7370.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4164.32,7419.99");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=79.64,7167.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=857.56,7082.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1560.68,7145.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3217.63,7085.50");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4497.26,7121.89");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4870.94,7197.38");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5482.04,7019.42");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5868.07,7103.46");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=351.94,6903.87");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2037.66,6877.30");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2411.94,6928.31");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2813.61,6918.29");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3983.83,6843.77");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=110.25,6640.39");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=580.68,6628.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1190.53,6663.35");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1694.05,6693.40");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3192.38,6589.20");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4842.37,6567.24");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5298.60,6713.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5727.15,6669.09");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1608.68,6290.18");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2247.03,6366.64");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2639.57,6393.44");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3566.85,6503.34");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4375.83,6497.87");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5498.54,6274.40");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5898.28,6344.81");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=46.69,6264.16");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=529.41,6182.28");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=957.48,6194.58");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1291.71,6057.48");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3039.67,6130.48");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4026.63,6054.81");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4552.95,6182.81");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4914.90,6221.89");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=537.17,5776.31");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1985.48,6019.19");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2574.96,5999.77");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3650.30,5883.14");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5245.32,5964.61");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=165.88,5609.85");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=942.56,5767.07");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1734.52,5761.51");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3006.85,5763.22");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4876.64,5552.22");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5999.98,5681.48");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1132.84,5382.59");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2280.60,5451.51");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3315.25,5510.73");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3690.62,5523.61");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4227.93,5506.95");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5538.50,5392.70");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5983.34,5279.23");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=280.76,5041.64");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=619.47,5187.24");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1563.76,5206.89");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2027.99,5192.73");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2689.90,5265.80");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3109.42,5103.48");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3929.23,5214.26");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4626.74,5131.38");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5235.26,5150.24");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3662.99,4826.22");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1094.35,4750.76");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1650.94,4744.40");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2228.23,4777.24");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4051.54,4673.01");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4467.95,4740.66");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4928.66,4702.44");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5336.20,4781.13");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5946.41,4759.58");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=154.87,4403.67");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=722.39,4533.58");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2077.22,4431.14");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2701.65,4318.02");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3116.72,4529.75");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3709.23,4382.55");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4102.82,4294.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5648.81,4493.24");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1260.31,4270.83");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1730.60,4157.00");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2405.73,4085.19");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3032.15,4140.90");
    }

}

