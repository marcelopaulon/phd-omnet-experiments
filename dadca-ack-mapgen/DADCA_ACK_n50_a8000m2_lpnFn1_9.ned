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

network DADCA_ACK_n50_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=10.79,83.03");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=129.38,46.10");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=76.63,187.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=94.02,254.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=158.17,188.53");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=222.44,178.27");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=314.64,148.44");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=379.24,134.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=462.25,92.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=537.29,38.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=599.53,37.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=537.96,105.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=633.74,165.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=679.50,97.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=566.31,165.76");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=613.05,244.03");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=670.91,266.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=747.97,258.78");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=833.35,220.94");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=822.49,284.96");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=909.93,297.34");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=825.72,130.86");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=739.83,128.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=786.56,391.61");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=845.02,358.72");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=648.62,362.80");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=712.94,362.79");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=537.19,365.88");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=467.75,360.26");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=502.86,309.50");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=426.91,299.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=525.03,222.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=304.21,288.14");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=378.30,247.28");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=322.88,209.19");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=256.91,230.37");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=286.79,68.83");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=443.20,154.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=350.81,68.51");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=222.09,309.56");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=284.02,381.37");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=196.09,251.70");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=258.02,438.47");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=215.46,497.19");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=127.10,468.48");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=115.78,393.91");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=42.34,347.64");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=23.64,413.91");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=128.40,309.35");
    }

}

