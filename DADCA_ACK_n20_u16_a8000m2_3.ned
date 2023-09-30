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

network DADCA_ACK_n20_u16_a8000m2_3
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
        @display("i=misc/sensor;p=636.85,7961.87");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1266.45,7766.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1647.86,7907.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2269.60,7851.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3871.91,7853.61");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4882.81,7945.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=117.57,7647.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=580.31,7614.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1610.69,7525.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2005.59,7575.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3044.22,7695.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4435.70,7653.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5494.23,7715.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=327.16,7314.02");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=998.52,7459.41");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3669.32,7409.95");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4874.47,7495.03");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=889.26,7086.72");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1393.82,7012.19");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2121.24,7189.88");
    }

}

