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
        @display("i=misc/sensor;p=2833.49,7989.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3442.81,7966.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4377.88,7863.65");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5369.21,7770.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=607.86,7719.84");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=938.59,7574.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1409.87,7643.90");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2332.08,7693.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3124.09,7732.25");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3516.78,7584.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3954.63,7711.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4774.47,7564.24");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5934.76,7725.33");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=82.73,7299.27");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1990.72,7468.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3319.06,7279.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3827.01,7382.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5394.85,7360.20");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=654.45,7251.03");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1102.24,7249.44");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1477.24,7078.04");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2063.38,7069.34");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2511.05,7223.99");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2869.42,7145.46");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4378.50,7233.53");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5844.96,7179.23");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=246.75,6838.67");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=717.31,6892.52");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1258.06,6787.06");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3617.32,6830.55");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4519.55,6905.93");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1673.76,6634.41");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2282.66,6609.42");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3148.80,6735.63");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4089.79,6759.85");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4891.57,6540.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=139.49,6465.47");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=734.94,6490.06");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2671.58,6510.97");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3580.77,6480.98");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4069.39,6324.71");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4436.48,6455.19");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5244.36,6511.94");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1037.04,6102.00");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1482.43,6060.55");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2283.88,6022.01");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2845.28,6161.73");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3446.13,6075.03");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4943.37,6107.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5789.75,6258.96");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=33.71,5904.47");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=691.24,5950.46");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3792.82,5947.54");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4394.61,5890.38");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5449.16,5988.44");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1874.92,5735.12");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2655.57,5702.64");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3073.62,5577.87");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4060.38,5703.76");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=264.19,5432.57");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1196.72,5462.64");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2356.45,5370.44");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3687.84,5481.36");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4389.91,5498.84");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4813.58,5466.94");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5253.74,5510.58");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5607.03,5402.19");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5987.99,5283.01");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=801.08,5259.74");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1435.54,5205.22");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1905.94,5272.36");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4029.62,5055.73");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5295.51,5086.89");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=567.91,4885.17");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1572.86,4873.95");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2234.15,4939.08");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2717.00,4944.67");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3295.02,4836.22");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4594.18,4879.81");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4969.48,4865.69");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5719.60,4964.79");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1161.02,4567.12");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2143.64,4568.55");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2620.52,4599.91");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=79.77,4500.06");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3255.80,4464.93");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3874.28,4530.73");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4438.53,4450.77");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5278.05,4492.98");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5746.72,4396.74");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=9.04,4095.98");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=728.43,4272.71");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1233.47,4210.94");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1672.64,4091.11");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2144.87,4213.36");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2767.84,4166.01");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3609.17,4264.71");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4092.21,4164.57");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4668.04,4169.05");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2212.76,3868.38");
    }

}

