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

network DADCA_ACK_n20_u16_a8000m2_0
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
        @display("i=misc/sensor;p=863.90,7934.30");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1314.43,7928.48");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2834.39,7942.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3752.42,7882.70");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4806.91,7992.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5589.72,7893.16");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=78.85,7696.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=561.41,7560.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1448.11,7597.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2175.16,7670.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3360.47,7720.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4015.00,7632.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=895.70,7455.11");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1233.51,7293.53");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2683.35,7379.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3099.54,7384.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3765.68,7340.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4440.30,7476.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5072.83,7496.71");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5680.70,7325.64");
    }

}

