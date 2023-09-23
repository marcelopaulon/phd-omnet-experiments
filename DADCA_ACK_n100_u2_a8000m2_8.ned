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

network DADCA_ACK_n100_u2_a8000m2_8
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
        @display("i=misc/sensor;p=904.08,7814.20");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1340.69,7862.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2321.10,7887.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2957.62,7863.84");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3421.49,7932.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3929.93,7996.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4378.41,7910.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4745.92,7914.70");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5203.89,7956.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5812.28,7799.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1633.01,7524.63");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2700.50,7527.69");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3519.00,7527.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4914.09,7578.44");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=470.97,7285.52");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1019.78,7333.05");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1963.26,7383.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3884.24,7441.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4309.31,7412.81");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5666.74,7449.52");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=79.87,7040.32");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1518.80,7065.82");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2314.21,7228.19");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2898.25,7173.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3325.16,7069.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3986.22,7060.27");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5247.60,7166.62");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5918.63,7154.40");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=612.37,6830.33");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2014.48,6989.46");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4503.29,6996.07");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5809.91,6799.70");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1174.69,6636.55");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1694.52,6598.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2107.42,6635.18");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2546.67,6689.59");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3665.76,6537.49");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4181.06,6756.16");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5002.75,6720.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=216.29,6363.06");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=826.04,6289.47");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2355.25,6385.88");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3100.64,6471.00");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4334.37,6355.62");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4857.65,6392.77");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5419.39,6439.29");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1256.93,6145.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1711.89,6024.96");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2676.30,6241.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2991.79,6060.24");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3400.13,6118.18");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3860.08,6195.86");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4624.35,6026.26");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5726.14,6138.02");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=787.22,5781.61");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2337.29,5901.63");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2687.01,5863.97");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5018.34,5920.47");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5959.77,5788.45");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=104.78,5641.98");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1299.72,5622.22");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2912.67,5555.41");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3209.11,5768.97");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4849.11,5545.08");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5602.53,5633.60");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=653.63,5371.55");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1879.16,5475.64");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3617.92,5497.09");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4296.99,5410.32");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5248.22,5371.01");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5943.30,5429.95");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1021.61,5256.62");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1374.16,5102.55");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2451.22,5208.01");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3068.82,5215.31");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4439.98,5082.06");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=415.02,4831.89");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=785.54,4869.38");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1619.99,4812.11");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2067.05,4992.69");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3475.81,4851.29");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3967.58,5015.18");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5492.08,4852.39");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=21.26,4754.01");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1128.78,4752.74");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2684.36,4691.16");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3097.30,4608.75");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4364.33,4682.28");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5928.32,4768.33");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=144.28,4310.28");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=572.71,4494.46");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1722.82,4393.28");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2391.19,4416.92");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3742.48,4360.66");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5060.08,4461.02");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=706.68,4076.38");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1305.26,4142.49");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2095.78,4222.94");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2621.22,4090.21");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3184.05,4122.73");
    }

}

