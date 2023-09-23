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

network DADCA_ACK_n50_u8_a8000m2_8
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
        @display("i=misc/sensor;p=2080.42,7757.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3266.73,7877.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5414.45,7974.09");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5831.13,7952.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=610.13,7713.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1081.88,7517.16");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1465.54,7687.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2879.17,7600.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3798.28,7704.24");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5740.09,7505.28");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=529.96,7270.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1975.10,7332.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2520.84,7479.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3244.22,7347.17");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3901.66,7321.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4480.44,7413.66");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4932.49,7445.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1287.68,7226.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1638.67,7156.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2958.46,7037.56");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5141.84,7156.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5496.06,7201.67");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5993.52,7061.58");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=40.91,6852.99");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2556.60,6997.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3417.36,6893.45");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4257.47,6869.18");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4746.99,6799.52");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5108.31,6774.82");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=492.79,6599.20");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=905.60,6698.78");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1391.33,6711.27");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1784.91,6694.24");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2947.38,6540.80");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5511.48,6734.61");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1265.92,6363.50");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2504.20,6431.53");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3344.78,6307.17");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3751.80,6368.37");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4594.58,6272.89");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5439.92,6373.27");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=404.47,6242.24");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=835.39,6106.12");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1495.61,6021.51");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1938.86,6128.51");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4237.04,6045.49");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5934.87,6199.60");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=167.08,5911.17");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2462.44,5878.62");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3365.39,5950.20");
    }

}

