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

network DADCA_ACK_n20_u4_a8000m2_9
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
        @display("i=misc/sensor;p=467.82,7910.99");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=860.32,7770.92");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1368.60,7988.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1747.21,7944.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2102.33,7967.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4383.98,7946.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5455.27,7945.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1222.83,7621.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1579.86,7601.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2121.94,7514.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2570.36,7606.25");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3179.45,7600.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5855.87,7647.10");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=338.39,7457.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=888.35,7385.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1776.70,7292.47");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3815.50,7431.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4335.75,7440.45");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4699.90,7502.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5135.71,7477.94");
    }

}

