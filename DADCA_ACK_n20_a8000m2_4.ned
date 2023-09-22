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

network DADCA_ACK_n20_a8000m2_4
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
        @display("i=misc/sensor;p=228.63,7815.42");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=734.35,7757.31");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2354.04,7938.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2861.94,7911.02");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3260.62,7945.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4403.04,7999.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4866.60,7785.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5268.77,7856.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5627.99,7854.97");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1081.35,7689.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1511.19,7552.54");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1847.39,7716.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2830.46,7545.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3536.20,7715.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3980.14,7665.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=509.86,7468.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2481.81,7284.83");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4317.86,7455.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5137.37,7372.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1105.10,7219.46");
    }

}

