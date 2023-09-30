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

network DADCA_ACK_n100_u4_a8000m2_4
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
        @display("i=misc/sensor;p=208.79,7880.18");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1050.83,7936.33");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1423.19,7909.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2472.54,7934.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2941.08,7761.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4297.28,7974.21");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=513.66,7670.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2004.21,7748.68");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2701.04,7505.92");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3507.68,7752.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5194.50,7723.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=827.87,7447.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1314.56,7338.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3399.05,7402.17");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4035.66,7432.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4689.38,7406.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5926.70,7500.46");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=87.35,7232.25");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=579.74,7078.81");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=975.90,7110.05");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1678.40,7166.95");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2950.96,7164.77");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4958.83,7060.42");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5499.22,7255.40");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5915.26,7123.87");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=225.30,6794.99");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1525.22,6767.42");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2085.88,6983.09");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2440.24,6964.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2966.79,6776.01");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3389.78,6771.57");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3857.07,6999.24");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4502.83,6952.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2637.67,6547.74");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4050.49,6600.67");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5242.27,6608.65");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5659.87,6531.43");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=419.12,6477.37");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1010.18,6281.10");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1431.20,6305.54");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1829.95,6443.92");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3623.89,6386.79");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4529.51,6314.24");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4935.34,6354.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2047.05,6063.48");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2361.34,6217.95");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2848.94,6070.79");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3209.12,6240.96");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5119.42,6029.39");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5842.70,6217.69");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=500.19,5893.17");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=979.17,5863.83");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1742.63,5795.24");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3920.89,5997.95");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5659.27,5806.54");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1293.28,5677.57");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2695.07,5530.34");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3163.62,5724.18");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3584.52,5722.92");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4519.91,5682.71");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4986.19,5703.83");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=105.57,5443.47");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1519.72,5373.94");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1952.11,5512.51");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3055.45,5366.97");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3747.87,5296.33");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5389.60,5465.31");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=477.28,5151.26");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=937.94,5156.50");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2214.59,5189.04");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4217.92,5119.04");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4613.72,5170.72");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5094.67,5102.02");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5777.10,5174.73");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1203.82,4883.66");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1868.73,4995.80");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3416.33,5006.62");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4041.52,4793.61");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4578.73,4811.97");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=314.22,4777.44");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=953.22,4545.34");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1903.54,4638.56");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2291.79,4763.49");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3090.47,4708.56");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3670.60,4759.49");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4977.45,4733.13");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5613.20,4734.07");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2126.40,4324.25");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3181.34,4308.52");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3601.51,4368.43");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4031.56,4374.61");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=266.82,4138.27");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=674.99,4180.90");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1418.62,4240.91");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2835.84,4242.65");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4405.76,4169.93");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4746.48,4258.82");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5255.62,4218.50");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5912.97,4258.93");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=949.64,3935.59");
    }

}

