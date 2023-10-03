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

network DADCA_ACK_n20_u4_a8000m2_4
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
        @display("i=misc/sensor;p=1338.35,7945.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1739.84,7811.60");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2651.90,7879.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3049.76,7971.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3571.35,7894.81");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5406.61,7906.79");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=434.02,7625.29");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1095.31,7538.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2895.29,7564.51");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4078.72,7672.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4775.37,7736.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1740.38,7360.40");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2262.39,7411.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3557.41,7280.11");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4583.49,7388.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4985.58,7453.71");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5356.35,7399.03");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5749.70,7319.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=687.88,7233.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2050.27,7080.57");
    }

}

