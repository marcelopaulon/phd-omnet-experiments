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

network DADCA_ACK_n100_u8_a8000m2_3
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
        @display("i=misc/sensor;p=1013.25,7981.56");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1563.20,7975.74");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2042.61,7906.73");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2636.22,7969.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3569.56,7859.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4516.23,7977.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=172.50,7733.81");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1892.30,7506.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2331.21,7525.28");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3954.34,7675.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4300.77,7586.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4663.41,7650.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=93.51,7274.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=667.75,7480.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1044.86,7435.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3015.82,7503.10");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3317.97,7259.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5201.58,7283.38");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=657.55,7013.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2155.09,7064.57");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2616.02,7197.62");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3788.30,7238.58");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4187.75,7058.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4605.89,7018.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1187.09,6908.99");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1792.84,6999.43");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3089.71,6873.95");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3652.37,6848.13");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5070.55,6957.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5845.51,7006.47");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=377.70,6739.07");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=816.59,6594.05");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1707.71,6655.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2337.35,6538.83");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2680.47,6664.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3005.49,6520.13");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4278.30,6581.21");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5157.03,6588.12");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=34.37,6371.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1424.94,6371.64");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4779.08,6312.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5609.95,6372.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5977.09,6321.52");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=690.46,6047.24");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1953.34,6066.27");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3250.79,6150.34");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3782.21,6104.04");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=8.92,5973.90");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1147.13,5842.60");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1520.01,5795.28");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2202.69,5785.27");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2626.15,5976.95");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3001.37,5846.92");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4989.50,5954.30");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5517.50,5913.52");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=502.48,5723.80");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1784.42,5559.76");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3294.81,5565.96");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4238.55,5618.80");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4612.48,5700.53");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5055.33,5572.28");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5929.30,5588.95");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=725.24,5420.29");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1138.48,5382.02");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2412.12,5464.83");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3727.10,5443.84");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4384.36,5279.54");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4782.31,5287.69");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=603.04,5066.92");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1732.01,5186.73");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2487.50,5119.73");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2971.16,5128.64");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3357.51,5055.61");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4102.88,5048.95");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5183.09,5076.36");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=966.78,4944.89");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2022.17,4787.15");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4747.95,4860.44");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5775.99,4997.19");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=503.17,4614.29");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1340.38,4708.02");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2409.08,4658.33");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3731.62,4732.80");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4420.11,4607.84");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5197.53,4644.44");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=989.19,4506.01");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2022.75,4436.13");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2768.87,4433.75");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3274.66,4496.30");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3666.90,4328.71");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4110.58,4294.80");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4988.89,4345.06");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5717.75,4441.65");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=162.86,4136.33");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=585.58,4070.07");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1441.98,4050.92");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2469.49,4121.87");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4623.38,4266.54");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1183.25,3811.28");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2068.53,4033.79");
    }

}

