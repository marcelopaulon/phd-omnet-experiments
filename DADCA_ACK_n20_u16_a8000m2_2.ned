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

network DADCA_ACK_n20_u16_a8000m2_2
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
        @display("i=misc/sensor;p=1923.19,7992.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3011.37,7833.53");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3633.40,7992.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4136.81,7866.93");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4946.75,7877.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=139.95,7566.64");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=631.75,7622.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=990.47,7701.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1340.35,7749.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2371.70,7708.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3653.54,7584.51");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4655.59,7515.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5340.14,7703.78");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1800.37,7460.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2722.08,7372.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3321.03,7466.30");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4254.80,7291.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5487.08,7375.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5843.71,7436.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=107.53,7197.54");
    }

}

