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

network DADCA_ACK_n50_u2_a8000m2_0
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
        @display("i=misc/sensor;p=1119.85,7985.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1718.48,7834.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2317.38,7853.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3249.41,7775.20");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3701.55,7990.06");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5151.46,7833.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5504.31,7972.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=606.32,7685.54");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1139.48,7612.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2009.97,7545.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2658.40,7632.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3637.83,7526.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=946.67,7310.32");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4760.80,7499.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5347.29,7431.90");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1483.02,7249.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2098.31,7077.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2556.68,7030.21");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2919.85,7253.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4289.34,7065.63");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4845.11,7025.04");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5267.36,7044.77");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5656.06,7197.61");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3413.89,6954.30");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3871.50,6804.53");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5696.76,6838.94");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1127.81,6677.04");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3102.86,6700.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4108.70,6536.81");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4589.72,6635.57");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5312.31,6614.56");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=850.50,6457.72");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1756.81,6450.90");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2194.50,6413.28");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3650.30,6437.99");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4896.96,6461.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=871.38,6074.48");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1385.23,6221.53");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2726.39,6165.30");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3239.24,6068.74");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3759.23,6088.70");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4346.17,6168.46");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5264.94,6062.60");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5807.76,6214.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1351.25,5861.13");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2061.34,5872.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4133.45,5806.41");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4845.93,5804.18");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=768.53,5568.33");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2936.93,5586.33");
    }

}

