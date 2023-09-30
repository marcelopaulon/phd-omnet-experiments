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

network DADCA_ACK_n20_u16_a8000m2_6
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
        @display("i=misc/sensor;p=75.83,7777.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=469.90,7839.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1669.04,7811.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2146.88,7792.31");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2923.34,7844.44");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3763.34,7859.28");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4416.17,7815.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1005.61,7585.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2490.85,7638.73");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3498.22,7603.01");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4884.03,7667.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5368.57,7740.63");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5811.05,7686.79");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=566.13,7459.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2992.65,7335.16");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3975.56,7383.94");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4611.87,7262.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5469.73,7368.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=67.10,7137.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=685.52,7077.90");
    }

}

