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
        @display("i=misc/sensor;p=19.71,7954.77");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=844.58,7842.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1559.19,7975.86");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2058.48,7908.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2993.94,7834.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=414.39,7678.08");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1293.94,7586.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1638.37,7516.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2501.17,7668.02");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3828.16,7555.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4459.11,7635.02");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4868.89,7628.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=93.03,7334.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=997.66,7366.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2036.84,7465.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3206.25,7502.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=568.09,7086.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1314.41,7085.04");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3637.74,7020.86");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4122.04,7044.94");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4462.21,7256.32");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4849.87,7159.34");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=120.52,6825.06");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1749.49,6933.94");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2272.10,6994.94");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2904.48,6937.13");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5383.42,6989.07");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5966.52,6772.28");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=514.26,6538.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=839.46,6703.75");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1421.47,6526.01");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2656.41,6585.90");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4367.92,6705.64");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4837.33,6601.21");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5400.82,6560.70");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2302.94,6418.53");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2959.53,6350.33");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3443.53,6360.32");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3897.62,6429.99");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4576.42,6315.31");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=530.00,6159.74");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1152.06,6194.79");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1567.21,6191.03");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5094.17,6166.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5739.80,6266.23");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1135.31,5825.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1861.08,5867.44");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2441.56,5932.87");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3006.80,5816.40");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3924.50,5804.96");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4770.63,5905.00");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5430.64,5867.19");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=775.61,5751.41");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3501.63,5645.85");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5674.41,5585.58");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=155.41,5477.81");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1460.51,5301.45");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2050.33,5472.72");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2659.55,5334.36");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4386.08,5353.67");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5128.45,5470.09");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=968.52,5172.94");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2186.11,5098.90");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3109.29,5227.79");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3462.08,5127.38");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3907.32,5081.88");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4828.58,5260.93");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5687.40,5036.13");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=171.38,4981.16");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=580.15,4941.09");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1769.84,4980.35");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2426.25,4835.09");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4733.58,4905.80");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5180.14,4822.99");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=858.76,4630.52");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1243.31,4645.90");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3118.56,4656.87");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3622.63,4625.98");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4255.46,4715.69");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5441.30,4577.60");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=214.14,4342.08");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1669.69,4486.80");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2125.02,4533.12");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2505.41,4456.70");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3928.12,4320.06");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4753.96,4401.17");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5795.10,4519.22");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=999.40,4226.26");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1522.79,4061.14");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2146.87,4129.25");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2494.41,4044.19");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3392.67,4073.44");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4346.59,4072.55");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5174.51,4208.36");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5670.79,4100.99");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=24.28,3881.26");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=472.72,3827.35");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3010.70,3798.13");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4880.18,3932.17");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=958.16,3648.75");
    }

}

