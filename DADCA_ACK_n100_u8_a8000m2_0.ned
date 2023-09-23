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

network DADCA_ACK_n100_u8_a8000m2_0
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
        @display("i=misc/sensor;p=668.56,7841.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1032.24,7784.15");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2770.84,7795.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3614.15,7995.88");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4904.56,7793.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2212.17,7656.82");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3076.03,7595.14");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3460.64,7679.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4377.20,7751.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5345.23,7628.50");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=425.44,7356.63");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1600.13,7322.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2751.61,7440.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3327.64,7278.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4306.59,7374.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4686.73,7382.51");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5033.89,7274.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=9.12,7197.10");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=769.15,7250.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1194.61,7228.67");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2072.64,7137.80");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2460.04,7057.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5534.26,7251.96");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=751.37,6830.15");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2877.93,6774.08");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3890.11,6975.75");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4225.33,6850.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4667.50,6997.95");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5133.20,6886.64");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5956.27,6977.92");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=319.50,6754.25");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1182.39,6566.91");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1539.30,6627.30");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1890.67,6520.59");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2346.61,6570.47");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3352.41,6668.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3928.87,6516.55");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4740.01,6525.68");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5302.35,6571.63");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5662.36,6570.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=22.35,6386.39");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2703.89,6451.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3110.78,6394.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3558.85,6295.71");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4329.71,6459.02");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=332.74,6223.80");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=701.61,6261.03");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1859.49,6103.71");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2268.81,6181.33");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4129.62,6170.39");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5519.47,6066.69");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=496.44,5801.12");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1257.58,5998.76");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2616.61,5919.83");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3009.17,5890.90");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4784.12,5968.61");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5875.38,5918.71");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1480.42,5596.87");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3977.20,5680.77");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4368.35,5742.00");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5131.31,5755.47");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=166.82,5437.59");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=922.34,5318.77");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1692.39,5299.04");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2136.72,5431.63");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2601.02,5469.97");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3193.02,5484.20");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4726.05,5451.09");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5575.48,5477.36");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1969.12,5067.86");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2511.02,5123.63");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2895.11,5108.23");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3610.01,5119.25");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4144.82,5207.16");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4804.61,5070.04");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=454.02,4863.53");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=846.76,4951.75");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1319.50,4968.36");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2303.99,4799.70");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3352.96,4844.52");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4178.90,4800.72");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5198.68,4896.04");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1688.14,4669.31");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2730.00,4715.10");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4576.81,4701.45");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5693.06,4715.22");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=890.17,4527.78");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2068.31,4495.95");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3108.98,4366.45");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3753.03,4485.70");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4344.39,4332.03");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5008.53,4345.22");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=109.45,4118.43");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=713.55,4146.64");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1349.54,4265.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2440.60,4199.64");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5501.89,4154.57");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1264.43,3833.52");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1789.94,3887.96");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2258.11,3830.98");
    }

}

