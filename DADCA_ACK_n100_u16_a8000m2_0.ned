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

network DADCA_ACK_n100_u16_a8000m2_0
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
        @display("i=misc/sensor;p=50.24,7908.90");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=594.78,7797.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1734.69,7824.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2125.34,7881.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2530.42,7856.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4291.81,7755.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4885.42,7842.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5387.13,7902.02");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1237.89,7677.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2365.96,7547.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3083.28,7675.12");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3804.28,7689.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5765.66,7567.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=307.20,7459.25");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1680.64,7289.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3628.81,7266.31");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5126.82,7469.02");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=758.04,7243.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1105.04,7090.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2222.40,7131.64");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2650.14,7024.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2986.71,7247.10");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4216.14,7212.45");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5382.92,7115.19");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=266.18,6793.87");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1823.22,6902.92");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3321.28,6839.53");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4875.07,6991.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5825.55,6835.38");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1341.85,6702.24");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5330.74,6690.54");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1087.54,6391.24");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2538.97,6488.85");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3083.15,6502.18");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4864.47,6334.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=47.37,6234.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=479.57,6144.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1888.73,6252.07");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2295.01,6232.79");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3475.57,6221.97");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4387.82,6200.95");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5488.66,6243.64");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=888.88,5915.41");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1404.88,5846.93");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1923.92,5839.69");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2333.56,5819.51");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2685.91,5884.38");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3221.26,5948.40");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3881.46,5825.91");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4876.43,5871.34");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5432.86,5777.79");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5885.74,5898.38");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=624.95,5616.78");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1089.69,5542.54");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4533.37,5657.98");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5148.47,5561.83");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1971.70,5472.94");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2776.03,5442.71");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3118.27,5343.91");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3987.43,5440.17");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4672.57,5282.51");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1055.81,5085.69");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1677.15,5245.00");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2407.11,5245.31");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5134.81,5190.83");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5496.29,5164.04");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=635.76,4955.96");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2064.48,4911.24");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2516.41,4817.39");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3070.90,4826.72");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3817.34,4816.82");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4278.21,4917.36");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=901.76,4620.56");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1655.21,4714.03");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3352.63,4606.77");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4716.08,4719.81");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5658.59,4615.68");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=504.62,4330.00");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1266.85,4421.68");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2101.51,4457.38");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3914.13,4316.62");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4305.35,4527.05");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4805.38,4363.92");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5315.28,4534.07");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=936.58,4160.56");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1785.74,4075.56");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2497.56,4247.66");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2906.69,4276.44");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5100.98,4166.40");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5527.67,4224.03");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5895.52,4251.74");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1226.15,3936.30");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3120.81,3847.62");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3478.70,3966.24");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3838.67,3794.43");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4420.39,3861.58");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=403.50,3681.90");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1055.14,3619.67");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1513.13,3713.69");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2025.82,3761.32");
    }

}

