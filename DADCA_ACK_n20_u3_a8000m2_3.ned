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

network DADCA_ACK_n20_u3_a8000m2_3
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
        @display("i=misc/sensor;p=9.61,7936.47");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2074.51,7765.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2382.39,7958.10");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2718.96,7849.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3175.88,7903.45");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=563.59,7647.28");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3589.22,7698.80");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4650.09,7574.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5283.64,7739.22");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5784.68,7588.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=254.74,7307.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=807.65,7372.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1307.81,7386.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1873.37,7345.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2755.72,7455.08");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3182.04,7494.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4140.00,7373.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5112.33,7412.24");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2374.34,7166.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2765.23,7097.03");
    }

}

