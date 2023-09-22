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

network DADCA_ACK_n20_a8000m2_9
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
        @display("i=misc/sensor;p=188.30,7986.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=601.93,7901.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4139.96,7904.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4603.89,7989.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5057.28,7768.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5555.36,7867.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5918.77,7770.75");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=51.84,7613.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=927.91,7690.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1315.23,7652.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1854.36,7584.46");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2765.72,7655.27");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3808.68,7648.86");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4731.60,7530.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=576.09,7443.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=986.13,7258.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2462.96,7253.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2911.59,7226.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3287.44,7268.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4165.10,7455.27");
    }

}

