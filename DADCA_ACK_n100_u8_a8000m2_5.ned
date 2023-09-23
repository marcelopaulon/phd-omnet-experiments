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
        @display("i=misc/sensor;p=557.21,7984.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=961.16,7786.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1291.64,7929.78");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1979.44,7988.94");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2560.19,7906.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3461.60,7890.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5227.93,7935.62");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=522.96,7570.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1686.19,7533.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2112.64,7641.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2631.89,7547.51");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3081.20,7701.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4012.23,7506.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4596.87,7745.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4947.66,7703.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5776.61,7709.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=41.47,7504.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1111.17,7404.70");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3198.84,7357.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4543.28,7277.41");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5458.66,7400.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=98.91,7099.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1495.30,7180.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2089.75,7251.63");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3655.41,7169.03");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5078.80,7019.41");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5905.42,7074.07");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=822.65,6875.26");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1860.58,6957.00");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2386.99,6990.67");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2921.92,6934.28");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4113.46,6868.98");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=320.55,6628.21");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1009.65,6516.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1809.49,6558.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2243.69,6634.53");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2901.99,6536.39");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3322.80,6619.97");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3734.87,6746.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4754.77,6738.61");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5486.87,6709.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1406.84,6472.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4159.02,6284.52");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4465.98,6454.31");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5869.06,6416.32");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=398.73,6130.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=952.52,6060.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2165.26,6146.65");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2602.97,6229.05");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3241.32,6175.30");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3768.42,6240.26");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4566.58,6033.36");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5061.84,6225.62");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5458.31,6102.12");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=58.05,5998.81");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1391.24,6018.75");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2535.04,5852.72");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5843.89,6004.74");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=490.21,5534.59");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1050.67,5570.24");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1865.70,5554.33");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2220.89,5685.26");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2792.64,5585.68");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3518.92,5668.32");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4129.75,5725.42");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4516.52,5584.70");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4914.28,5553.01");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5456.72,5641.14");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=65.37,5396.45");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1393.76,5342.32");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2452.81,5371.39");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5213.61,5334.11");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=746.27,5230.50");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1803.77,5171.85");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2167.95,5084.57");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2783.21,5141.52");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3820.56,5118.24");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4383.05,5135.70");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5629.20,5255.38");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=621.74,4886.44");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1016.98,5001.13");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3339.72,4960.28");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4666.35,4843.76");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5879.10,4880.64");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=270.88,4777.84");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1510.78,4769.17");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2418.03,4660.29");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2823.90,4777.08");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4325.59,4712.69");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5226.35,4743.82");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=297.10,4413.66");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=661.35,4446.70");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1010.75,4394.35");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1526.28,4328.48");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1998.14,4394.51");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3044.24,4437.22");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3516.45,4488.79");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3931.19,4534.62");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4959.15,4461.52");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5893.16,4525.38");
    }

}

