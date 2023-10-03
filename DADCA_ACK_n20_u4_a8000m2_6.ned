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

network DADCA_ACK_n20_u4_a8000m2_6
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
        @display("i=misc/sensor;p=148.09,7912.76");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=630.39,7827.78");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1094.26,7867.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1890.02,7859.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2342.06,7960.05");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3364.47,7919.61");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3737.16,7780.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4835.73,7793.60");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5457.21,7889.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=83.04,7523.45");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1454.51,7658.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2550.81,7544.45");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2977.86,7675.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4276.36,7698.63");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5708.68,7595.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=405.99,7353.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2121.95,7477.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3289.34,7464.93");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5336.25,7332.32");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5980.43,7310.34");
    }

}

