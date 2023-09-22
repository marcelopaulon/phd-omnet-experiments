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

network DADCA_ACK_n50_a8000m2_6
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
        @display("i=misc/sensor;p=141.47,7992.21");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=219.74,7997.68");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=352.30,7981.99");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=644.04,7971.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=806.48,7985.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=174.25,7931.41");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=259.50,7946.11");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=318.24,7920.68");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=424.05,7940.74");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=517.36,7939.98");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=603.27,7914.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=749.49,7917.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=847.71,7923.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=930.05,7946.25");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=62.69,7908.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=132.69,7874.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=453.26,7886.69");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=678.71,7911.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=18.20,7847.33");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=87.11,7830.38");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=199.17,7859.69");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=317.07,7855.66");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=413.66,7832.66");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=510.96,7855.24");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=855.80,7860.98");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=578.97,7821.79");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=645.47,7825.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=760.87,7825.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=913.81,7787.28");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=94.05,7741.23");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=161.40,7780.50");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=270.38,7774.55");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=356.47,7779.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=467.06,7773.66");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=563.01,7760.57");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=733.76,7744.61");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2.40,7714.82");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=385.73,7717.49");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=445.57,7705.61");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=507.86,7720.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=652.48,7725.73");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=806.88,7734.73");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=87.58,7680.53");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=175.34,7686.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=299.66,7683.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=577.15,7667.73");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=889.93,7666.40");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=955.13,7683.71");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=165.96,7620.77");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=264.92,7622.20");
    }

}

