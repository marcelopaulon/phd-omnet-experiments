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

network DADCA_ACK_n20_u8_a8000m2_7
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
        @display("i=misc/sensor;p=605.12,7810.11");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1150.86,7989.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1638.81,7783.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1995.95,7863.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2464.55,7890.19");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3091.39,7768.94");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5563.86,7938.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=172.89,7714.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3795.92,7658.86");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4331.86,7563.11");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5007.76,7627.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=143.65,7297.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=618.32,7453.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1142.16,7370.62");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2154.31,7360.35");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4635.72,7361.18");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5436.28,7376.83");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1621.76,7084.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2288.82,7025.78");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2693.62,7204.89");
    }

}

