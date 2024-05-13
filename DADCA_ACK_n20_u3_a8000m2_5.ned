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

network DADCA_ACK_n20_u3_a8000m2_5
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
        @display("i=misc/sensor;p=0.68,7761.43");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=728.56,7879.48");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1694.88,7963.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2542.83,7927.47");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3269.23,7954.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3922.49,7980.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1215.19,7699.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1750.42,7602.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2984.52,7705.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4175.89,7678.15");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4555.04,7545.41");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5527.49,7661.15");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=159.87,7443.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1961.37,7302.48");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2440.84,7502.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3319.17,7438.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3728.46,7454.40");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5136.50,7495.37");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5967.57,7491.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=770.04,7193.70");
    }

}

