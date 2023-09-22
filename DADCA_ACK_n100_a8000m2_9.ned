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

network DADCA_ACK_n100_a8000m2_9
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
        @display("i=misc/sensor;p=100.94,7893.03");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3433.96,7927.30");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4395.49,7800.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5199.36,7957.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1462.01,7645.44");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1936.63,7554.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3080.51,7648.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3745.09,7615.99");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4656.94,7502.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=211.34,7395.77");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2362.83,7481.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2755.69,7448.07");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4136.13,7318.46");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5343.32,7399.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=718.99,7090.75");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1203.36,7134.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1750.13,7090.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2550.01,7032.24");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3279.37,7086.24");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3848.80,7062.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4789.58,6983.74");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=189.45,6923.73");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4470.11,6728.67");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5150.11,6745.36");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=576.20,6634.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=967.67,6495.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2032.05,6533.03");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2768.63,6616.53");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3684.59,6606.92");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5581.40,6657.06");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5948.77,6505.83");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=160.13,6311.85");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1434.26,6353.20");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2890.48,6198.34");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3309.59,6248.33");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4259.30,6394.12");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4637.08,6226.09");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5403.27,6201.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=725.68,5991.27");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1099.67,6132.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2034.89,5970.73");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2465.44,6075.75");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4972.62,6062.45");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1446.23,5922.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3123.07,5854.33");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3754.87,5894.45");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4246.55,5829.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=32.25,5496.86");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=417.31,5637.83");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1019.42,5501.25");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1846.88,5433.77");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2410.72,5656.27");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2844.59,5602.98");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3330.20,5541.16");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4608.66,5654.55");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5311.13,5657.37");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1423.11,5262.40");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2727.10,5179.12");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3133.81,5168.31");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3893.16,5409.84");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5554.53,5332.83");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=72.38,4898.32");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=486.09,5053.31");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3516.61,5075.00");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4429.66,5071.48");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5237.16,5046.87");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1105.61,4746.51");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1483.57,4760.46");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2047.34,4794.64");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3187.64,4798.75");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3692.19,4736.44");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4144.25,4757.01");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4904.26,4711.38");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5495.31,4663.02");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=362.42,4632.88");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=855.80,4419.90");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2636.21,4488.86");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=235.98,4275.70");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1788.68,4292.96");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3051.52,4254.98");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3701.36,4332.70");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4539.26,4179.56");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5230.39,4179.22");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=614.24,3870.13");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1335.44,4049.75");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2254.12,4028.91");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2719.56,4021.73");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3956.74,4011.95");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5767.41,4007.95");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=76.69,3805.39");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1563.60,3682.60");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1984.68,3668.07");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3440.91,3841.98");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4259.22,3683.68");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5534.08,3708.64");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=199.25,3454.87");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1197.38,3412.44");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2719.87,3401.06");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3346.27,3365.65");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3782.42,3348.56");
    }

}

