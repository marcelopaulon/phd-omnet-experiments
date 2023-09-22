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

network DADCA_ACK_n20_a8000m2_5
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
        @display("i=misc/sensor;p=2984.61,7956.90");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3667.95,7774.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4454.86,7991.18");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5149.73,7912.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5987.00,7832.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=156.80,7732.81");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5396.12,7579.11");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1391.96,7334.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2314.36,7327.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2753.32,7468.98");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3286.78,7326.86");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3972.93,7423.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4527.70,7476.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4898.62,7277.07");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5912.38,7379.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=239.70,7024.24");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=833.20,7197.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2076.04,7009.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2664.27,7035.56");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3845.52,6997.84");
    }

}

