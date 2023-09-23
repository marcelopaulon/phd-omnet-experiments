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

network DADCA_ACK_n100_u2_a8000m2_3
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
        @display("i=misc/sensor;p=133.84,7796.84");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=975.24,7935.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2760.74,7854.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3467.99,7775.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4151.07,7857.04");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4784.59,7960.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1160.15,7509.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2086.77,7660.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4741.37,7542.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5393.26,7622.68");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=273.08,7268.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=717.46,7435.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1656.74,7478.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2671.03,7382.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3157.95,7503.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3851.22,7411.12");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4333.68,7398.11");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5952.93,7259.28");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=953.79,7026.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1468.08,7157.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2249.06,7206.21");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3304.26,7161.82");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4204.56,7065.45");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5065.72,7037.26");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1756.52,6845.30");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2553.84,6843.51");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3054.03,6855.63");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3750.68,6843.83");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5501.95,6987.91");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=349.16,6689.38");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=723.21,6754.70");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2081.67,6571.01");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4532.09,6751.95");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5997.39,6756.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1387.94,6453.29");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1761.06,6268.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2391.29,6357.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2998.29,6357.19");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3414.76,6444.30");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4123.45,6466.84");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4546.04,6275.84");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4975.79,6393.98");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5597.50,6312.61");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=776.58,6214.22");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2147.59,6024.52");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2523.50,6022.36");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3582.54,6136.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=422.45,5919.07");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1233.90,5905.34");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1589.05,5908.95");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3071.55,5948.30");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5068.38,5884.09");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=799.34,5680.86");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2021.19,5553.30");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2683.70,5586.09");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4076.63,5756.95");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4731.45,5627.95");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5655.81,5656.69");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=484.43,5514.16");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1468.57,5347.30");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3634.18,5408.56");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4431.38,5351.07");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5253.63,5312.79");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5960.15,5392.21");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=68.85,5177.83");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1039.32,5242.29");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2448.00,5248.04");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3320.48,5200.68");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4174.84,5102.32");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4832.94,5163.51");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5576.90,5138.92");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=419.20,4884.99");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1004.60,4866.08");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1794.19,4895.13");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2161.54,4910.10");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2956.18,5003.17");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3498.41,4887.63");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5896.63,4823.97");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1437.36,4746.97");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2185.13,4557.24");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2638.69,4614.50");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3100.00,4620.73");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4321.81,4728.02");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4846.54,4614.69");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5290.50,4749.96");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1340.49,4349.49");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1789.08,4388.36");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3660.47,4462.02");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4025.45,4531.13");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5463.71,4377.59");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=708.57,4256.39");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2562.74,4259.67");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3341.57,4086.84");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4393.10,4066.73");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4731.28,4227.03");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5078.57,4115.69");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=236.53,3981.56");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1222.42,3964.39");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1797.62,3940.39");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2151.71,3928.21");
    }

}

